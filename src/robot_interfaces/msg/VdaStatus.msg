# /vda5050/status — агент VDA 5050 для Compass. При изменении и раз в секунду (reliable, transient_local, depth 1).
string connection                # ONLINE | OFFLINE — связь агента с брокером fms
string order_id                  # пусто — order'ов ещё не было; после завершения остаётся последний
uint32 order_update_id
bool order_active                # order выполняется (робот не idle по VDA 6.6.8)
string last_node_id
bool driving
bool paused
bool new_base_request            # робот на decision point, ждёт продления маршрута от fms
StatusError[] errors             # ошибки VDA: code — errorType, message — errorDescription
