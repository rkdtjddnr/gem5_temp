import re

# 로그 데이터 (여기에 직접 문자열 입력 가능)
log_data = """
DTA RX mbuf addresses:
0x20130ed80, 0x20130f700, 0x201310080, 0x201310a00, 0x201311380, 0x201311d00, 
[LOG], 11377282226769, DTA_RX_JOB_REQ_RECV
DTA RX Job ID: 13
DTA RX mbuf addresses:
0x201312680, 0x201313000, 0x201313980, 0x201314300, 0x201314c80, 0x201315600, 0x201315f80, 0x201316900, 
[LOG], 11377282245750, DTA_RX_JOB_REQ_RECV
DTA RX Job ID: 13
DTA RX mbuf addresses:
0x201317280, 0x201317c00, 0x201318580, 0x201318f00, 0x201319880, 0x20131a200, 0x20131ab80, 0x20131b500, 
[LOG], 11377282264731, DTA_RX_JOB_REQ_RECV
DTA RX Job ID: 13
DTA RX mbuf addresses:
0x20131be80, 0x20131c800, 0x20131d180, 0x20131db00, 0x20131e480, 0x20131ee00, 0x20131f780, 0x201320100, 
[LOG], 11377282283712, DTA_RX_JOB_REQ_RECV
DTA RX Job ID: 13
DTA RX mbuf addresses:
0x201320a80, 0x201321400
"""

lines = log_data.split("\n")  # 줄 단위로 분할
mbuf_addresses = []
parsing = False

for line in lines:
    line = line.strip()

    # "DTA RX mbuf addresses:"가 나오면 parsing 시작
    if line.startswith("DTA RX mbuf addresses:"):
        parsing = True
        # 뒤에 있는 주소들을 추출
        addresses = [int(match, 16) for match in re.findall(r'0x[0-9a-fA-F]+', line)]
        mbuf_addresses.extend(addresses)
    elif parsing:
        # 주소가 계속 나오는 부분
        addresses = [int(match, 16) for match in re.findall(r'0x[0-9a-fA-F]+', line)]
        if addresses:
            mbuf_addresses.extend(addresses)
        else:
            parsing = False  # 주소가 끝나면 종료


# 각 address에서 계산된 값 저장
mbuf_addr_set = [f'0x{addr:X}' for addr in mbuf_addresses]
mbuf_first_cacheline_set = [f'0x{addr - 256:X}' for addr in mbuf_addresses]
mbuf_second_cacheline_set = [f'0x{addr - 192:X}' for addr in mbuf_addresses]

# 4개씩 출력 포맷 함수
def format_c_array(name, values):
    formatted = ",\n    ".join([", ".join(values[i:i+4]) for i in range(0, len(values), 4)])
    return f"uint64_t {name}[] = {{\n    " + formatted + "\n};\n"

# C 스타일 배열로 변환하여 출력
print(format_c_array("mbuf_addr_set", mbuf_addr_set))
print(format_c_array("mbuf_first_cacheline_set", mbuf_first_cacheline_set))
print(format_c_array("mbuf_second_cacheline_set", mbuf_second_cacheline_set))