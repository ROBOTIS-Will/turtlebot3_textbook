이 값은 장치가 움직이고 있는지 여부를 알려줍니다. 만약 Present Velocity(615)의 절대값이 Moving Threshold(17)에 설정된 값보다 크다면, Moving(610)은 '1'로 설정됩니다.  
그렇지 않은 경우에는 Moving(610)의 값은 '0'으로 설정됩니다.  
하지만 Profile이 진행중일 때, 즉 Goal Position(596) 명령을 수행하는 중에는 Present Velocity(615)와 무관하게 '1'로 설정됩니다.

| 값 | 설명     |
| :---: | :------------- |
| 0 | 움직임이 감지되지 않음 |
| 1 | 움직임이 감지되었거나, Profile이 진행중인 경우(Goal Position(596) 명령을 수행하는 중) |
