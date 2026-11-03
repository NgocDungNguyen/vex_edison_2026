# Ngân hàng lý do: vì sao StarLight dùng C++ thay vì code khối (Block)

> **Chỉ dành cho mentor.** Đây là *kho gợi ý* để mentor đặt câu hỏi và để học sinh **tự chọn, tự thử, tự đo, tự viết lại bằng lời của mình**. Đừng đưa nguyên văn cho học sinh chép vào sổ. Giám khảo sẽ hỏi trực tiếp, và một lý do không có bằng chứng (số liệu / ảnh / log) chỉ là lời nói.
> Nếu học sinh tìm ý từ tài liệu, video, người khác hoặc công cụ AI, phải ghi vào Credit Summary.

## Cách dùng: mỗi lý do đi theo 3 bước

1. **Chọn** lý do thật sự đúng với đội (đừng chọn cả danh sách).
2. **Kiểm chứng** bằng một thí nghiệm nhỏ, ghi số liệu hoặc chụp ảnh.
3. **Viết** vào bảng "Vì sao C++…" trong Phần 9, kèm trang bằng chứng.

Bảng trong sổ (Phần 9) có sẵn các cột: Lý do — Tụi mình đã thử / đọc gì — Bằng chứng — Trang — Nguồn ý tưởng.

## A. Lý do KỸ THUẬT (và cách kiểm chứng)

| # | Gợi ý lý do | Cách đội tự kiểm chứng |
|---|---|---|
| A1 | Điều khiển chính xác hơn: tự đặt hằng số, dùng hàm, truy cập cấu hình động cơ chi tiết hơn | Chạy cùng 1 đường 10 lần ở cả Block và C++, đo sai lệch (cm / độ) |
| A2 | Tái sử dụng code bằng **hàm** (ví dụ `driveFor(distance, speed)`) thay vì kéo thả lặp lại khối | Đếm số dòng / số khối để làm cùng một bài; số chỗ phải sửa khi đổi 1 thông số |
| A3 | **Biến và hằng số** đặt tên có nghĩa (`SHOOT_SPEED`), sửa 1 chỗ đổi cả chương trình | Đổi 1 tham số, ghi thời gian sửa |
| A4 | **Vòng lặp, điều kiện, mảng, cấu trúc** gọn hơn khi chiến thuật phức tạp | So sánh độ dài/độ rối chương trình autonomous |
| A5 | Đọc cảm biến liên tục, **phản hồi (feedback)**: dùng PID đơn giản / kiểm soát tốc độ | Đo độ lệch khi đi thẳng có và không có hiệu chỉnh |
| A6 | Chạy **nhiều tác vụ song song** (thread/task): ví dụ tay gắp và bánh xe cùng lúc | Thử cùng lúc / tuần tự, bấm giờ |
| A7 | Kiểm soát **thời gian chạy**, độ trễ phản hồi tay cầm | Ghi thời gian từ bấm nút → robot phản ứng |
| A8 | Gỡ lỗi bằng **in giá trị ra màn hình Brain / console** | Chụp ảnh log khi tìm một lỗi thật |
| A9 | **Cấu trúc dự án rõ ràng**: file cấu hình, DK (driver), Auto riêng | Ảnh cây thư mục + ghi lý do tách file |
| A10 | Cấu hình thiết bị/cổng (port map) minh bạch ở một nơi duy nhất | Đối chiếu Brain port map ở sổ với code |
| A11 | Dùng **thư viện / API VEX** đầy đủ hơn (tài liệu chính thức) | Ghi tên hàm mới mà Block chưa có, link tài liệu |
| A12 | **Điều khiển tay cầm tuỳ biến**: dead-zone, đảo chiều, tốc độ tuỳ chọn, chế độ chính xác | Ghi thay đổi cảm giác lái, ý kiến người lái |
| A13 | Quản lý **phiên bản** bằng Git: biết ai sửa gì, quay lại bản cũ | Ảnh lịch sử commit, nhật ký phiên bản code (có sẵn trang trong sổ) |

## B. Lý do HỌC TẬP & TƯƠNG LAI (kết nối ra ngoài cuộc thi)

| # | Gợi ý lý do | Ghi chú cho mentor |
|---|---|---|
| B1 | C++ là ngôn ngữ thật được dùng trong robot, ô tô, game, nhúng, hệ thống | Để học sinh tự tìm ví dụ thật và ghi nguồn |
| B2 | Chuyển lên các ngôn ngữ khác (Python, Java, C#) dễ hơn khi đã quen cú pháp, biến, hàm, kiểu dữ liệu | Hỏi: "em muốn học gì tiếp theo?" |
| B3 | Làm quen **công cụ chuyên nghiệp**: trình soạn thảo, build, Git, đọc lỗi trình biên dịch | Kết nối với phần PM: quy trình công nghiệp |
| B4 | Học **tư duy thuật toán** (chia nhỏ bài toán, thử, sửa) thay vì chỉ ghép khối | Cho học sinh vẽ lưu đồ trước khi code (trang Thuật toán) |
| B5 | Kỹ năng **đọc tài liệu kỹ thuật tiếng Anh** (đúng phương châm thi VEX quốc tế) | Ghi từ khoá tiếng Anh học được |
| B6 | Có thể **làm việc chung code** nhiều người: review code, chia file, tránh sửa chồng | Ghi lại một lần review code của nhau |
| B7 | Thi tin học / Olympic / lập trình thi đấu sau này (C++ phổ biến) | Chỉ nêu nếu đội thật sự quan tâm |
| B8 | Phù hợp lộ trình ngành kỹ thuật phần mềm, robot, điện tử, AI | Hỏi các em nghe ai/chỗ nào nói vậy, ghi nguồn |
| B9 | Cảm giác **làm chủ**: hiểu từng dòng code của robot mình, trả lời được phỏng vấn sâu | Mỗi bạn chọn một đoạn code và giải thích bằng lời |

## C. Góc nhìn "sáng tạo" để mentor khơi gợi (không phải để chép)

- **Block giống sơ đồ bản nhạc có sẵn nốt; C++ giống tự sáng tác.** Hỏi học sinh: "Còn ví dụ nào của riêng em?"
- **Code là tài liệu**: tên biến/hàm tốt giúp người ngoài đội hiểu robot làm gì (nhìn từ phía giám khảo / đội kế cận).
- **Sổ + code cùng kể một câu chuyện**: mỗi bản code (v1, v2…) gắn với một thí nghiệm và một quyết định.
- **Di sản đội**: các bạn lớp dưới đọc code năm sau có hiểu không? Ghi chú code như thư gửi tương lai.
- **Rủi ro biết trước**: nêu luôn điểm yếu của C++ (xem phần D); tự nhận hạn chế khiến lập luận đáng tin hơn.

## D. Đánh đổi thật — phải ghi trung thực (giám khảo rất thích)

| Khó khăn khi dùng C++ | Cách đội xử lý (để học sinh tự điền) |
|---|---|
| Lỗi cú pháp, lỗi biên dịch lúc đầu nhiều | Đọc thông báo lỗi, ghi lỗi hay gặp và cách sửa |
| Học lâu hơn khi mới bắt đầu | Chia sẻ kiến thức cho nhau, mỗi tuần 1 chủ đề nhỏ |
| Dễ gõ sai, khó trực quan hơn khối | Dùng lưu đồ trước khi code, đặt tên rõ |
| Chỉ vài bạn hiểu code → rủi ro cho đội | Review code chéo, tài liệu hoá, luân phiên ai làm code |
| Thời gian thi gấp, sửa nhanh ở sân | Chuẩn bị tham số chỉnh ở đầu file, có bản dự phòng |

## E. Một bài so sánh đáng làm (trang "So sánh Block và C++ trên cùng một bài")

Chọn **một bài nhỏ** (ví dụ: đi thẳng 1 m, quay 90°, gắp 1 bean bag, đổ vào ô). Học sinh tự làm bằng Block và bằng C++, rồi điền bảng: thời gian viết, số dòng/khối, độ chính xác (số liệu), dễ đọc lại sau 1 tuần, dễ tìm lỗi. **Kết luận phải dựa trên số liệu của chính đội**, kể cả khi Block thắng ở một mục.

## F. Ghi nguồn và trung thực

- Mọi đoạn code mượn (VEX mẫu, đội khác, video, AI) → Credit Summary + chú thích ngay tại trang code.
- Giám khảo có thể hỏi một bạn bất kỳ giải thích một đoạn code; hãy luyện cho từng bạn nói được vì sao nó nằm đó.
- Code cũ của Zenith (thư mục `zenith/`) là điểm xuất phát nếu được dùng: phải ghi rõ "từ robot Zenith, chỉnh sửa lại thế nào".
