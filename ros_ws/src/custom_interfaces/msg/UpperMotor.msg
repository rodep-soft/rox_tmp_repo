bool is_throwing_on
# 射出はこれがTrueになるとrelayで駆動する
# これは押すとFalse, Trueが永続的に切り替わるのではなく、
# 押したときだけTrueになり、on()になるという話

bool is_ejection_on

uint8_t elevation_mode 
# 0 -> 下
# 1 -> 上
# 2 -> STOP