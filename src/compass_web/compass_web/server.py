"""Веб-сервер Compass на роботе — вместо nginx (ROS_INTERFACES.md §5).

  /                 статика UI (ui_dir — собранный `npm run build`); неизвестный путь → index.html
  /rosbridge        WebSocket: кадры как есть пересылаются в rosbridge (он слушает только localhost)
  /files/maps/      файлы карт             — чтение
  /files/logs/      журнал compass_core    — чтение
  /files/ros_log/   логи ROS 2             — чтение
  /files/uploads/   файлы для импорта карт — только PUT

Каталог на чтение отдаётся JSON-листингом, как у nginx (autoindex_format json):
[{ name, type: file | directory, mtime, size }]. Параметры — compass_bringup/config/compass_web.yaml.
"""
import json
import os
from email.utils import formatdate

import rclpy
from rclpy.signals import SignalHandlerOptions
import tornado.ioloop
import tornado.web
import tornado.websocket

MB = 1024 * 1024
MAX_WS_MESSAGE = 64 * MB  # карта 3000×3000 в CBOR — ~9 МБ, с запасом


def inside(root, path):
    """Абсолютный путь внутри root или None — чтобы `../` не выводил за каталог."""
    root = os.path.realpath(root)
    target = os.path.realpath(os.path.join(root, path))
    return target if target == root or target.startswith(root + os.sep) else None


def listing(directory):
    out = []
    for name in sorted(os.listdir(directory)):
        try:
            st = os.stat(os.path.join(directory, name))
        except OSError:
            continue  # битая ссылка
        entry = {'name': name, 'mtime': formatdate(st.st_mtime, usegmt=True)}
        if os.path.isdir(os.path.join(directory, name)):
            entry['type'] = 'directory'
        else:
            entry.update(type='file', size=st.st_size)
        out.append(entry)
    return out


class UiHandler(tornado.web.StaticFileHandler):
    """Статика UI. Пути, которого нет на диске, — index.html: это маршруты Vue (/drive, /maps)."""

    def validate_absolute_path(self, root, absolute_path):
        if not os.path.isfile(absolute_path):
            absolute_path = os.path.join(root, 'index.html')
        return super().validate_absolute_path(root, absolute_path)

    def set_extra_headers(self, path):
        # assets/ — с хэшем в имени, не меняются; index.html — всегда свежий, иначе новый UI не подхватится.
        if path.startswith('assets/') and os.path.basename(self.absolute_path) != 'index.html':
            self.set_header('Cache-Control', 'public, max-age=31536000, immutable')
        else:
            self.set_header('Cache-Control', 'no-cache')


class NoUiHandler(tornado.web.RequestHandler):
    def initialize(self, ui_dir):
        self.ui_dir = ui_dir

    def get(self, _path):
        self.set_status(503)
        self.set_header('Content-Type', 'text/plain; charset=utf-8')
        self.finish(f'Compass UI не собран: нет {self.ui_dir}/index.html (npm run build, параметр ui_dir)\n')


class FilesHandler(tornado.web.StaticFileHandler):
    """Файлы на чтение; каталог — JSON-листингом."""

    async def get(self, path, include_body=True):
        target = inside(self.root, path)
        if target is None or not os.path.exists(target):
            raise tornado.web.HTTPError(404)
        if os.path.isdir(target):
            self.set_header('Content-Type', 'application/json')
            self.set_header('Cache-Control', 'no-cache')
            return self.finish(json.dumps(listing(target)))
        await super().get(path, include_body)

    def set_extra_headers(self, path):
        self.set_header('Cache-Control', 'no-cache')

    def compute_etag(self):
        # ETag StaticFileHandler считает по файлу; у листинга каталога файла нет.
        return super().compute_etag() if getattr(self, 'absolute_path', None) else None


class UploadHandler(tornado.web.RequestHandler):
    """PUT файла для импорта карты; размер ограничивает max_upload_mb."""

    def initialize(self, root):
        self.root = root

    def put(self, path):
        target = inside(self.root, path)
        if target is None or target == os.path.realpath(self.root):
            raise tornado.web.HTTPError(404)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, 'wb') as f:
            f.write(self.request.body)
        self.set_status(201)


class RosbridgeProxy(tornado.websocket.WebSocketHandler):
    """Браузер ⇄ rosbridge: кадры как есть (текст — JSON, бинарные — CBOR карты)."""

    def initialize(self, url, log):
        self.url = url
        self.log = log
        self.upstream = None

    def check_origin(self, origin):
        return True  # UI открывают по любому адресу робота (IP, имя) — как было за nginx

    async def open(self):
        # Пока open не вернулся, Tornado не зовёт on_message — кадры браузера не потеряются.
        try:
            self.upstream = await tornado.websocket.websocket_connect(self.url, max_message_size=MAX_WS_MESSAGE)
        except Exception as e:  # rosbridge не запущен или упал
            self.log.warning(f'rosbridge {self.url} недоступен: {e}')
            self.close(1011, 'rosbridge unavailable')
            return
        tornado.ioloop.IOLoop.current().spawn_callback(self.pump)

    async def pump(self):
        while True:
            msg = await self.upstream.read_message()
            if msg is None:
                break  # rosbridge закрыл соединение
            try:
                await self.write_message(msg, binary=isinstance(msg, bytes))
            except tornado.websocket.WebSocketClosedError:
                break
        self.close()

    async def on_message(self, message):
        if self.upstream:
            await self.upstream.write_message(message, binary=isinstance(message, bytes))

    def on_close(self):
        if self.upstream:
            self.upstream.close()


def main():
    # SIGINT (Ctrl+C, ros2 launch) — обычный KeyboardInterrupt: Tornado его и ждёт.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('compass_web')
    log = node.get_logger()

    def param(name, default):
        return node.declare_parameter(name, default).value

    port = param('port', 8080)
    address = param('address', '0.0.0.0')
    ui_dir = os.path.expanduser(param('ui_dir', ''))
    data_dir = os.path.expanduser(param('data_dir', '/var/lib/robot'))
    ros_log_dir = os.path.expanduser(param('ros_log_dir', '') or os.environ.get('ROS_LOG_DIR') or '~/.ros/log')
    rosbridge_url = param('rosbridge_url', 'ws://127.0.0.1:9090')
    max_upload_mb = param('max_upload_mb', 200)

    routes = [
        (r'/rosbridge', RosbridgeProxy, {'url': rosbridge_url, 'log': log}),
        (r'/files/uploads/(.+)', UploadHandler, {'root': os.path.join(data_dir, 'uploads')}),
        (r'/files/maps/(.*)', FilesHandler, {'path': os.path.join(data_dir, 'maps')}),
        (r'/files/logs/(.*)', FilesHandler, {'path': os.path.join(data_dir, 'logs')}),
        (r'/files/ros_log/(.*)', FilesHandler, {'path': ros_log_dir}),
    ]
    if os.path.isfile(os.path.join(ui_dir, 'index.html')):
        routes.append((r'/(.*)', UiHandler, {'path': ui_dir}))
    else:
        log.warning(f'UI не найден: {ui_dir}/index.html — соберите npm run build и проверьте ui_dir')
        routes.append((r'/(.*)', NoUiHandler, {'ui_dir': ui_dir}))

    app = tornado.web.Application(
        routes, websocket_max_message_size=MAX_WS_MESSAGE, websocket_ping_interval=20)
    app.listen(port, address, max_body_size=max_upload_mb * MB)
    log.info(f'Compass: http://{address}:{port}  UI: {ui_dir}  rosbridge: {rosbridge_url}')
    try:
        tornado.ioloop.IOLoop.current().start()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
