<!-- 
AX,EX,DX,RX Series : Shutdown [18], Torque Enable [24]
X-Series: Shutdown [63], Torque Enable [64]
PRO-Series: Shutdown [48], Torque Enable [562]
PRO+ Series: Shutdown [63], Torque Enable [512]
Revision: PRO+ > DYNAMIXEL-P.
-->


장치는 동작 중에 발생하는 위험 상황을 감지하여 스스로를 보호할 수 있습니다.  
각 Bit의 기능은 ‘OR’의 논리로 적용되기 때문에 중복 설정이 가능합니다.  
즉, {% if page.product_group=='dxl_ax' or page.product_group=='dxl_dx' or page.product_group=='dxl_ex' or page.product_group=='dxl_rx' or page.product_group=='dxl_mx' %} [Shutdown(18)] {% elsif page.product_group=='dxl_pro' %} [Shutdown(48)] {% else %} [Shutdown(63)] {% endif %} 이 ‘0x05’ (2 진수: 00000101)로 설정되었을 경우, Input Voltage Error(2 진수 : 00000001)와 Overheating Error(2 진수 : 00000100)가 발생하는 것을 모두 감지할 수 있습니다.  
위험 상황이 감지되면, {% if page.product_group=='dxl_ax' or page.product_group=='dxl_dx' or page.product_group=='dxl_ex' or page.product_group=='dxl_rx' or page.product_group=='dxl_mx' %} [Torque Enable(24)] {% elsif page.product_group=='dxl_pro_a' or page.product_group=='dxl_p' %} [Torque Enable(512)] {% elsif page.product_group=='dxl_pro' %} [Torque Enable(562)] {% else %} [Torque Enable(64)] {% endif %} 값이 ‘0’으로 변경되고 모터 출력은 0 [%]가 됩니다.  
위험 상황이 감지된 후에는 Reboot을 하지 않는 한, {% if page.product_group=='dxl_ax' or page.product_group=='dxl_dx' or page.product_group=='dxl_ex' or page.product_group=='dxl_rx' or page.product_group=='dxl_mx' %} [Torque Enable(24)] {% elsif page.product_group=='dxl_pro_a' or page.product_group=='dxl_p' %} [Torque Enable(512)] {% elsif page.product_group=='dxl_pro' %} [Torque Enable(562)] {% else %} [Torque Enable(64)] {% endif %}을 ‘1’(Torque ON)로 설정할 수 없습니다.  
{% if page.product_group=='dxl_ax' or page.product_group=='dxl_dx' or page.product_group=='dxl_ex' or page.product_group=='dxl_rx' or page.product_group=='dxl_mx' %}{% else %} 제어기는 Status Packet의 Error 필드에 Hardware Error Bit(0x80)이 설정되었는지를 확인하거나, {% if page.product_group=='dxl_pro' %} [Hardware Error Status(892)] {% elsif page.product_group=='dxl_pro_a' or page.product_group=='dxl_p' %} [Hardware Error Status(518)] {% else %} [Hardware Error Status(70)] {% endif %} {% endif %} 을 통해서 현재 상태를 확인할 수 있습니다.  
{% if page.product_group=='dxl_ax' or page.product_group=='dxl_dx' or page.product_group=='dxl_ex' or page.product_group=='dxl_rx' or page.product_group=='dxl_mx' %} [Shutdown(18)] {% elsif page.product_group=='dxl_pro' %} [Shutdown(48)] {% else %} [Shutdown(63)] {% endif %} 에서 감지할 수 있는 위험 상황은 아래 표와 같습니다.

{% if page.product_group=='dxl_ax' or page.product_group=='dxl_dx' or page.product_group=='dxl_ex' or page.product_group=='dxl_rx' or page.product_group=='dxl_mx' %}
|  Bit  |        항목         | 설명                                                                                     |
|:-----:|:-------------------:|:-----------------------------------------------------------------------------------------|
| Bit 7 |          0          | -                                                                                        |
| Bit 6 |  Instruction Error  | 정의되지 않은 Instruction이 전송된 경우, 또는 reg_write명령없이 Action명령이 전달된 경우 |
| Bit 5 |   Overload Error    | 모터의 최대 출력으로 제어할 수 없는 하중이 지속적으로 적용되는 경우                      |
| Bit 4 |   CheckSum Error    | 전송된 Instruction Packet의 ChecklSum이 맞지 않을 경우                                   |
| Bit 3 |     Range Error     | 해당 Address의 값의 범위를 벗어난 값을 Instruction Packet으로 보내는 경우                |
| Bit 2 |  OverHeating Error  | 내부 온도가 설정된 동작 온도 범위를 벗어난 경우                                          |
| Bit 1 |  Angle Limit Error  | 적용한 Goal Position이 설정한 CW/CCW Angle Limit 범위를 벗어난 경우                      |
| Bit 0 | Input Voltage Error | 인가된 전압이 설정된 동작 전압 범위를 벗어났을 경우                                      |
{% elsif page.product_group=='dxl_pro' or page.product_group=='dxl_pro_a' or page.product_group=='dxl_p' %}
|  Bit  |              명칭               | 내용                                                                                   |
|:-----:|:-------------------------------:|:---------------------------------------------------------------------------------------|
| Bit 7 |                -                | 미사용, 항상 '0'                                                                       |
| Bit 6 |                -                | 미사용, 항상 '0'                                                                       |
| Bit 5 |     Overload Error(초기값)      | 모터의 최대 출력으로 제어 할 수 없는 하중이 지속적으로 적용되는 경우                   |
| Bit 4 | Electrical Shock Error(초기값)  | 전기적으로 회로가 충격을 받거나, 입력 전력이 부족해서, 모터가 정상동작하지 못하는 경우 |
| Bit 3 |   Motor Encoder Error(초기값)   | 모터의 엔코더가 동작하지 않을 경우                                                     |
| Bit 2 |        Overheating Error        | 내부 온도가 설정된 동작 온도 범위를 벗어난 경우                                        |
| Bit 1 | Motor Hall Sensor Error(초기값) | 모터의 홀센서 값이 정상 범위를 벗어났을 경우                                           |
| Bit 0 |       Input Voltage Error       | 인가된 전압이 설정된 동작 전압 범위를 벗어났을 경우                                    |
{% else %}
|  Bit  |              명칭              | 상세 설명                                                                                |
|:-----:|:------------------------------:|:-----------------------------------------------------------------------------------------|
| Bit 7 |               -                | 미사용, 항상 '0'                                                                         |
| Bit 6 |               -                | 미사용, 항상 '0'                                                                         |
| Bit 5 |     Overload Error(초기값)     | 최대 출력으로 제어할 수 없는 하중이 지속적으로 발생한 경우                               |
| Bit 4 | Electrical Shock Error(초기값) | 전기적으로 회로가 충격을 받았거나, 입력 전력이 부족해서 모터가 정상 동작하지 못하는 경우 |
| Bit 3 |      Motor Encoder Error       | 모터의 엔코더가 동작하지 않는 경우                                                       |
| Bit 2 |   OverHeating Error(초기값)    | 내부 온도가 설정된 동작온도 범위를 벗어난 경우                                           |
| Bit 1 |               -                | 미사용, 항상 '0'                                                                         |
| Bit 0 |      Input Voltage Error       | 인가된 전압이 설정된 동작전압 범위를 벗어난 경우                                         |
{% endif %}

{% if page.product_group=='dxl_xw540' %}
{% capture shutdown_01 %}
**참고** : Shutdown이 발생할 경우 장치를 **장치를 REBOOT** 시켜 주세요 
-  H/W REBOOT : 전원을 껐다 켜주세요.
-  S/W REBOOT : REBOOT Instruction을 사용할수 있습니다. [Protocol 2.0](/docs/kr/dxl/protocol2/#reboot)을 참고해주세요.
{% endcapture %}
<div class="notice">{{ shutdown_01 | markdownify }}</div>

{% elsif page.product_group=='dxl_ax' or page.product_group=='dxl_dx' or page.product_group=='dxl_ex' or page.product_group=='dxl_rx' or page.product_group=='dxl_mx' %}
**참고** : Shutdown이 발생하면, **1초 주기로 LED가 점멸합니다.** 전원을 껏다 다시 켜주세요 (Reboot).  
{: .notice}
{% else %}
{% capture shutdown_01 %}
**참고** :
{% if page.product_group=='dxl_pro' or page.product_group=='dxl_pro_a' or page.product_group=='dxl_p' %}1. Shutdown이 발생하면 **전기적 브레이크(Dynamic brake) 상태로 전환**됩니다.{% else %}{% endif %}
2. Shutdown이 발생하면 **1초 주기로 LED가 점멸** 합니다.{% if page.product_group=='dxl_pro' or page.product_group=='dxl_pro_a' or page.product_group=='dxl_p' %}{% else %}(**펌웨어 버전 41 이상**) {% endif %}
3. Shutdown이 발생하면 다음과 같은 방법으로 **장치를 REBOOT** 시켜 주세요.
-  H/W REBOOT : 전원을 껐다 켜주세요.
-  S/W REBOOT : REBOOT Instruction을 사용할수 있습니다. [Protocol 2.0](/docs/kr/dxl/protocol2/#reboot)을 참고해주세요.
{% endcapture %}
<div class="notice">{{ shutdown_01 | markdownify }}</div>
{% endif %}


[Shutdown(18)]: #shutdown 
[Shutdown(48)]: #shutdown 
[Shutdown(63)]: #shutdown
[Torque Enable(24)]: #torque-enable
[Torque Enable(64)]: #torque-enable
[Torque Enable(512)]: #torque-enable
[Torque Enable(562)]: #torque-enable
[Hardware Error Status(70)]: #hardware-error-status
[Hardware Error Status(518)]: #hardware-error-status
[Hardware Error Status(892)]: #hardware-error-status
