사용자는 이 기능을 이용해, 필요한 컨트롤 테이블을 모아서 이용할 수 있습니다.  
Indirect Address Table에 특정 주소를 세팅하면, Indirect Data Table은 특정 주소와 동일한 기능을 가지게 됩니다.  
예를 들어, Indirect Address 1(49)에 563을 쓰고, Indirect Data 1(634)에 255를 쓰게 되면, 붉은 색LED에 불이 들어옵니다. LED RED(563)의 값 또한 255로 쓰여있습니다.  
또한, LED RED(563)에 값을 쓰면, Indirect Data 1의 값 또한 똑같이 변합니다. Indirect Address에 특정 주소를 세팅하게 되면, Indirect Data는 그것과 동일한 테이블이 됩니다.  
주의해야 할 점은 2byte 이상의 길이를 가진 Control Table을 Indirect Address로 설정할 때입니다.  
Control Table Item의 모든 byte를 Indirect Address로 세팅 해주어야 정상 동작합니다.  
예를 들어, Indirect Data 2를 Goal Position(596)으로 사용하고 싶을 땐, 아래와 같이 세팅해야 합니다.

#### 예제 1
1 바이트 LED(563)를 Indirect Data 1(634)에 할당하기.
1. Indirect Address 1(49) : RED LED의 주소값인 `563`으로 변경.
2. Indirect Data 1(634)을 '1'로 변경 : LED Red(563)값 또한 '1'로 변경되며 붉은색 LED가 켜짐.
3. Indirect Data 1(634)을 '0'로 변경 : LED Red(563)값 또한 '0'로 변경도며 LED가 꺼짐.

#### 예제 2
4 바이트 길이의 Goal Position(596)를 Indirect Data 2(635)에 할당하기 위해서는 반드시 연속된 4 바이트를 모두 할당해야 함.
1. Indirect Address 2(51) : 값을 Goal Position의 첫번째 주소인 '596'로 변경.
2. Indirect Address 3(53) : 값을 Goal Position의 두번째 주소인 '597'로 변경.
3. Indirect Address 4(55) : 값을 Goal Position의 세번째 주소인 '598'로 변경.
4. Indirect Address 5(57) : 값을 Goal Position의 첫번째 주소인 '599'로 변경.
5. Indirect Data 2에 250,961(0x0003D451)을 할당 : Goal Position(596) 역시 250,961(0x0003D451)로 변경됨.

| Indirect Data 주소 | Goal Position 주소 | 저장된 HEX 값 |
| :----------: | :-------------: | :-------------: |
| 635 | 596 | 0x51 |
| 636 | 597 | 0xD4 |
| 637 | 598 | 0x03 |
| 638 | 599 | 0x00 |

**참고** : 2바이트 이상의 데이터를 Indirect Address에 할당하기 위해서는 모든 데이터의 주소를 '예제 2'와 같이 Indirect Address에 할당해주어야 합니다.
{: .notice}

**참고** : Indirect Address 29 ~ 56와 Indirect Data 29 ~ 56는 프로토콜 2.0에서만 접근할 수 있습니다.
{: .notice}
