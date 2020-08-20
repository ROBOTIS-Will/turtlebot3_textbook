장치의 동작 모드를 설정합니다. 각 동작 모드마다 특성이 다르기 때문에, 구현하려는 시스템에 적합한 동작 모드를 설정하시기 바랍니다.

|값|동작 모드| 세부 설명     |
| :---- | :------------------------------ | :------------------------------------------- |
| 0 | 전류제어 모드 | 전류 제어 모드 속도와 위치는 제어하지 않고 전류(토크)를 제어합니다.<br />지령된 전류(토크)만 제어하는 시스템이나 그리퍼 또는 상위 제어기(위치, 속도 등)를 별도로 구성할 경우에 유용합니다. |
| 1 | 속도제어 모드 | 속도를 제어하는 모드 입니다.<br />기존제품의 바퀴 모드(무한회전)과 동일합니다.<br />바퀴형태의 로봇에 유용합니다. |
| 3(초기값) | 위치제어 모드  |  위치를 제어하는 모드 입니다.<br />기존제품의 관절 모드와 동일합니다.<br />Max Position Limit(48), Min Position Limit(52)에 의해서 동작 범위가 제한됩니다.<br />1회전 내에서 구동하는 다관절 로봇에 유용합니다.  |
|  4  |  확장 위치제어 모드(Multi-turn)  |  위치를 제어하는 모드 입니다.<br />기존제품의 다중 회전 모드와 동일합니다.<br />동작 범위는 총 512회전(-256 ~ 256[rev]) 입니다.<br />다수의 회전(멀티턴)이 필요한 로봇의 손목 부위나 컨베이어시스템 또는 추가 감속기가 필요한 시스템에 유용합니다.  |
|  5  |  전류기반 위치제어 모드  |  위치와 전류(토크)를 제어합니다.<br />동작 범위는 총 512회전(-256 ~ 256[rev]) 입니다.<br />위치와 전류를 동시에 제어할 필요가 있는 다관절 로봇이나 그리퍼에 유용합니다.  |
|  16  |  PWM 제어 모드 (Voltage Control Mode)  |  PWM 출력을 직접 제어합니다. (Voltage Control Mode)  |

{% capture group_notice_01 %}
**참고** : 동작 모드가 변경될 때 제어기의Gain(PID, Feedforward)은 동작 모드에 적합하게 초기화 됩니다. 또한 프로파일 생성기와 제한값들 역시 초기화 됩니다.
1. [Profile Velocity(112)](#profile-velocity112), [Profile Acceleration(108)](#profile-acceleration108) : '0'으로 초기화
2. [Goal PWM(100)](#goal-pwm100), [Goal Current(102)](#goal-current102) : [PWM Limit(36)](#pwm-limit36), [Current Limit(38)](#current-limit38)으로 초기화
3. 전류기반 위치 제어 모드 : 별도의 Position Gain(PID)과 [PWM Limit(36)](#pwm-limit36) 값으로 재설정 됩니다.  

변경된 Position Gain(PID)과 [PWM Limit(36)](#pwm-limit36) 값은 컨트롤테이블을 통해서 확인할 수 있습니다.
{% endcapture %}
<div class="notice">{{ group_notice_01 | markdownify }}</div>

{% capture opmode_notice_02 %}
**참고** : PWM이란 Pulse Width Modulation(펄스 폭 변조)의 약자로 펄스의 폭(PWM Duty)을 변경시키는 변조방식을 뜻합니다. 펄스의 폭을 변경하여 모터에 공급되는 평균 전압을 제어하는 용도로 사용됩니다.
1. PWM 모드는 다이나믹셀 DYNAMIXEL [AX](/docs/kr/dxl/ax/ax-12w/#cw-compliance-margin) 및 [RX](/docs/kr/dxl/rx/rx-10/#moving-speed-32) 시리즈의 바퀴모드와 유사합니다.
2. [Goal PWM(100)](#goal-pwm100)을 이용하여 모터에 공급되는 전압을 제어하세요.
{% endcapture %}
<div class="notice">{{ opmode_notice_02 | markdownify }}</div>

{% include kr/dxl/control_table_opmode_note.md %}
