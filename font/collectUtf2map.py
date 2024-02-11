import re
import argparse

def findZhTwChar(string):
    # 包含了中文字符以及全形標點符號的範圍
    pattern = r'[\u4e00-\u9fff\u3000-\u303f\uff00-\uffef]'
    return set(re.findall(pattern, string))
def charToUtf8Hex(char):
    return char.encode('utf-8').hex()
def charToUnicodeHex(char):
    return format(ord(char), 'X')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--files", nargs='+', help="File to be scanned for Chinese characters.")
    parser.add_argument("-o", "--output", nargs='?', help="Output file for the map required by bdfconv.")
    args = parser.parse_args()

    characters = set()  # 使用 set 來儲存獨特的中文字符
    for fileName in args.files:
        try:
            with open(fileName, 'r', encoding='utf-8') as file:
                for line in file:
                    characters.update(findZhTwChar(line))  # 更新字符集合
        except IOError:
            print(f"檔案 [{fileName}] 無法打開。")

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as outputFile:
            outputFile.write(f"32-128,\n")
            for char in sorted(characters):  # 對字符進行排序以獲得一致的輸出
                outputFile.write(f"${charToUnicodeHex(char)},\n")
    else:
        # 如果沒有指定輸出檔案，則在終端輸出結果
        for char in sorted(characters):
            print(f"${charToUnicodeHex(char)},")
    exit(0)