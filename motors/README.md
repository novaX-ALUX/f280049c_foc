# motors/ —— 电机参数变体（变化轴 2）
每个电机一个头文件，集中放该电机的 FOC 参数：极对数、Rs、Ls(d/q)、磁链/Ke、
额定电流/电压、最大转速、惯量等。换电机=换这里一个头，不动 app/ 和 boards/。

现状：电机参数暂时仍在 boards/esc6288_revA/drivers/include/user.h（SDK 单电机模板）。
下一步：把其中"电机相关"宏抽到 motor_<型号>.h，board 只留"硬件标定"宏，二者解耦。
（可用 is05_motor_id 自动辨识 Rs/Ls/磁链后回填。）
