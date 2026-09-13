import struct


def ReadFloat(byte_ar, adr=0) -> None:
    try:
        val = struct.unpack_from("<f", byte_ar, adr)[0]
        return val

    except Exception as e:
        print("ReadFloat except " + str(e).replace("\n", ""))
        return None


def read_hex_data(file_path, start_line, num_lines, start_address, byte_length):
    # 指定した行数のみを読み込む
    hex_data = []
    with open(file_path) as f:
        for i, line in enumerate(f):
            if i >= start_line and i < start_line + num_lines:
                hex_data.extend(
                    line.strip().split(),
                )  # 各行のデータをスペース区切りで分割し、リストに追加
            elif i >= start_line + num_lines:
                break  # 指定した行数を超えたら終了

    # 指定した範囲のデータを取得
    selected_data = hex_data[start_address : start_address + byte_length]

    outdata = []
    for data in selected_data:
        outdata.append(int(data, 16))

    for i in range(len(outdata) // 4):
        tmp_outdata = outdata[i * 4 : (i + 1) * 4]
        tmp_byte_ar = bytearray(tmp_outdata)
        val = ReadFloat(tmp_byte_ar)
        print(val)

    return bytearray(outdata)


if __name__ == "__main__":
    file_path = "log/damp_hex_c0.txt"  # 16進データが格納されているファイル
    start_line = 1  # 読み出しを開始する行番号 (0から始まるインデックス)
    num_lines = 1  # 読み出す行数
    start_address = 0  # 読み出し開始位置
    byte_length = 256  # 読み出すバイト長

    # 読み出した文字列を表示
    print(read_hex_data(file_path, start_line, num_lines, start_address, byte_length))
