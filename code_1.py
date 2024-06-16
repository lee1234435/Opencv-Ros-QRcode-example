import qrcode

# 데이터 설정
data = "x=0.5,y=0.5,w=1.0"

# QR 코드 생성
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_L,
    box_size=10,
    border=4,
)
qr.add_data(data)
qr.make(fit=True)

# 이미지 저장
img = qr.make_image(fill_color="black", back_color="white")
img.save("qr_code_image.png")
