bool is_system_ready
# システム準備完了フラグ（INIT -> STOPPED遷移用）

bool is_throwing_on
# 射出はこれがTrueになるとrelayで駆動する
# これは押すとFalse, Trueが永続的に切り替わるのではなく、
# 押したときだけTrueになり、on()になるという話

bool is_ejection_on
# これも動作については上のis_throwing_onと同じ挙動

uint8 elevation_mode 
# 0 -> 下
# 1 -> 上
# 2 -> STOP