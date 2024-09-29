#define PIN_LED 7 // LED가 연결된 핀을 GPIO 7번으로 설정
unsigned int count, toggle; // 전역 변수 선언
// count: 루프 반복 때마다 증가, toggle: 0과 1로 LED 상태 표시

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  count = 0;
  toggle = 1;  // LED 초기 상태 on으로 변경
  digitalWrite(PIN_LED, toggle); // LED 초기 상태 on
}

void loop() {
  if (count == 0) {
    toggle = 0;  // LED 끄기
    digitalWrite(PIN_LED, toggle); // LED 끄기
    delay(1000); // 1초 대기
    count++; // count 증가
  } 
  else if (count > 0 && count <= 11) { // count가 1~11일 때
    toggle = toggle_state(toggle); // LED 상태 변경
    digitalWrite(PIN_LED, toggle); 
    delay(100); // 1000/10 = 100ms 
    count++; // count 증가
  } 
  else if (count == 12) {
    toggle = 1;  // 마지막에 LED 켜기
    digitalWrite(PIN_LED, toggle); // LED 켬
    delay(100); // LED 켜는 것 확실히 대기
    count++; // count 증가
  } 
  else if (count > 12) {
    while (1) {} // 무한 루프
  }
}

int toggle_state(int toggle) {
  return (toggle == 1) ? 0 : 1; // toggle이 1이면 0 반환, 0이면 1 반환
}


// 회로는 교안의 사진과 똑같이 연결했으나 코드를 맞게 작성 후 실행 시 결과과 반대로 나왔습니다.
// 그래서 로직을 반대로 설정하여 원하는 출력 결과가 나오도록 수정했습니다.
