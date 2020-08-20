현재 적용되는 하중을 의미합니다.  
이 값의 범위는 0 ~ 2,047이며, 단위는 약 0.1 [%]입니다.  
0 ~ 1,023 범위의 값은 CCW 방향으로 하중이 작용한다는 의미입니다.  
1,024 ~ 2,047 범위의 값은 CW 방향으로 하중이 작용한다는 의미입니다.  
즉, 10번째 bit가 방향을 제어하는 direction bit가 되며, 1,024는 0과 같습니다.  
예를 들어, 값이 512이면 CCW 방향으로 최대 출력 대비 약 50 [%]로 하중이 감지된다는 의미입니다.  

| Bit | 15 ~ 11  | 10 | 9 ~ 0|
| :----: | :---: | :---: | :---: |
| 값 | 0 | 하중 방향 | 데이터 (하중 비율)|

**참고** : CCW 하중 : 하중 방향 비트(Bit 10) = 0, CW 하중 : 하중 방향 비트(Bit 10) = 1
{: .notice}

**참고** : 현재하중은 토크센서 등을 이용하여 측정된 값이 아니라 내부 출력 값을 기반으로 유추된 값 입니다. 따라서 무게나 토크를 측정하는 용도로는 부정확 할 수 있습니다. 해당 관절에 가해지는 힘의 방향과 크기를 예측하는 용도로 사용하시기를 권장합니다.
{: .notice}