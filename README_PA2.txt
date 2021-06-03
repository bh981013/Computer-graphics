display() 함수의 변화
1.doAni의 추가로 6개의 control point가 추가되었는지 확인 후 animate()함수를 실행
2.cow2wld_list는 최대 6개의 각기다른 cow2wld를 담고있는 list임. 이 list에서 cow2wld를 뽑아 drawCow를 실행.

onMouseButton() 함수의 변화
1. 만약 마우스 down 일어나면, 현재 상태를 저장하여 Vertical dragging을 준비함
2.만약 마우스 up이 일어나면, 마우스 커서의 변화가 있는지를 확인함. 변화가 있었다면 V_dragging이 일어난것이므로, H_dragging을 준비함. 만약 변화가 없었다면, 그자리에서 control point를 확정지음.

onMouseDrag() 함수의 변화
1. 만약 V_Drag 상태라면, x축을 normal vector로 갖는 평면과 z축을 normal vector로 갖는 평면이 교차하는 선 위에서 움직임이 일어남.

animate()함수의 추가
1. animate함수가 처음 실행된다면, 필요한 resource들을 할당해줌.
2. t값을 [0,1]사이로 유지시켜줌
3.B-spline에 해당하는 matrix와 control point를 조합해 현재 위치를 계산함.
4.현재 향하는 방향을 계산해 cow가 rotation할 angle을 결정함.
5.만약 3번 돌았으면 처음상태로 다시 돌아감.