"""Тесты соответствия ~/fleet-manager/tests в симуляции compass_sim — по одному.

Каждый тест уводит робота на несколько метров вперёд, за прогон он упёрся бы в стену.
Поэтому перед каждым тестом робот order'ом через агента возвращается в (-5, 0) курсом
на восток (карта склада TB4: впереди ~12 м свободно).

Робот должен быть локализован и в AUTOMATIC. Запуск — питоном из venv тестов:
  VDA_BROKER_HOST=127.0.0.1 VDA_MANUFACTURER=compass VDA_SERIAL=sim-001 \
    ~/fleet-manager/tests/.venv/bin/python run_tests.py test_orders.py test_horizon.py
"""
import os
import subprocess
import sys
import time
import uuid

TESTS = os.path.expanduser("~/fleet-manager/tests")
sys.path.insert(0, TESTS)

from fms_client import FmsClient, RobotTarget, is_idle  # noqa: E402
from order_builder import RouteNode, build_order  # noqa: E402

target = RobotTarget(host=os.environ.get("VDA_BROKER_HOST", "localhost"), port=1883, interface="amr",
                     major="v3", manufacturer=os.environ["VDA_MANUFACTURER"],
                     serial=os.environ["VDA_SERIAL"], username=None, password=None)


def reposition():
    fms = FmsClient(target, client_id=f"reposition-{uuid.uuid4().hex[:6]}")
    fms.connect()
    fms.send_raw("instantActions", '{"headerId":1,"timestamp":"2026-01-01T00:00:00.000Z","version":"3.0.0",'
                 '"manufacturer":"x","serialNumber":"y","actions":[{"actionId":"sr-%s",'
                 '"actionType":"stateRequest","blockingType":"NONE"}]}' % uuid.uuid4().hex[:6])
    s = fms.wait_message("state", lambda m: True, timeout=10)
    if s.get("paused"):
        fms.send_raw("instantActions", '{"headerId":2,"timestamp":"2026-01-01T00:00:00.000Z","version":"3.0.0",'
                     '"manufacturer":"x","serialNumber":"y","actions":[{"actionId":"up-%s",'
                     '"actionType":"stopPause","blockingType":"NONE"}]}' % uuid.uuid4().hex[:6])
    if not is_idle(s):
        fms.send_raw("instantActions", '{"headerId":3,"timestamp":"2026-01-01T00:00:00.000Z","version":"3.0.0",'
                     '"manufacturer":"x","serialNumber":"y","actions":[{"actionId":"co-%s",'
                     '"actionType":"cancelOrder","blockingType":"NONE"}]}' % uuid.uuid4().hex[:6])
        time.sleep(2)
        s = fms.latest("state")
    p = s["mobileRobotPosition"]
    if abs(p["x"] + 5) < 0.3 and abs(p["y"]) < 0.3 and abs(p["theta"]) < 0.3:
        fms.close()
        return
    # Сначала в (-6, 0), потом коротким участком на восток в (-5, 0): робот развернётся на месте.
    for x, y in [(-6.0, 0.0), (-5.0, 0.0)]:
        tag = uuid.uuid4().hex[:6]
        s = fms.latest("state")
        p = s["mobileRobotPosition"]
        nodes = [RouteNode(f"rp{tag}_0", p["x"], p["y"]), RouteNode(f"rp{tag}_1", x, y)]
        fms.send_order(build_order(f"reposition-{tag}", 0, nodes, released=2, map_id=p["mapId"]))
        fms.wait_state(lambda st: st["lastNodeId"] == nodes[-1].node_id and not st["driving"], 90,
                       what=f"robot at ({x}, {y})")
        time.sleep(1)
    fms.close()


ids = subprocess.run([f"{TESTS}/.venv/bin/python", "-m", "pytest", "--collect-only", "-q", *sys.argv[1:]],
                     cwd=TESTS, capture_output=True, text=True).stdout.splitlines()
ids = [i for i in ids if "::" in i]
for test_id in ids:
    reposition()
    r = subprocess.run([f"{TESTS}/.venv/bin/python", "-m", "pytest", "-q", "-p", "no:cacheprovider", "--tb=line",
                        test_id], cwd=TESTS, capture_output=True, text=True)
    last = [line for line in r.stdout.splitlines() if line.strip()][-1]
    print(f"{test_id}: {last}", flush=True)
    if r.returncode not in (0, 5):
        print("\n".join(line for line in r.stdout.splitlines() if line.startswith(("E ", "/", "FAILED", "ERROR")))[:2000],
              flush=True)
