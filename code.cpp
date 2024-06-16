#include <iostream>
#include <cstring>
#include <qrencode.h>

int main() {
    // QR 코드에 담을 데이터 입력
    const char *data = "https://www.example.com";

    // QR 코드 생성
    QRcode *qr = QRcode_encodeString(data, 0, QR_ECLEVEL_L, QR_MODE_8, 1);

    // QR 코드 이미지 출력
    for (int y = 0; y < qr->width; ++y) {
        for (int x = 0; x < qr->width; ++x) {
            std::cout << (qr->data[y * qr->width + x] & 1 ? "##" : "  ");
        }
        std::cout << std::endl;
    }

    // QR 코드 메모리 해제
    QRcode_free(qr);

    return 0;
}
