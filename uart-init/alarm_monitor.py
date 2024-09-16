import tkinter as tk
import serial
import serial.tools.list_ports
import struct
import threading

# 初始化串口變量
ser = None

def connect_serial(port):
    """
    根據選擇的串口端口初始化 UART 連接。
    :param port: 串口端口名稱
    """
    global ser
    if ser:
        ser.close()
    ser = serial.Serial(port, 9600, timeout=1)
    ser.flushInput()
    ser.flushOutput() 
    ser.write(b'\xAA\x82\x01\x2D')  # 發送初始數據

def update_temperature(square_id, temperature):
    """
    更新對應正方形的溫度顯示
    :param square_id: 小正方形的 ID
    :param temperature: 接收到的溫度數據
    """
    if square_id == 0:
        # 更新大正方形的溫度顯示
        large_label.config(text=f"ID: 0\nTemp: {temperature}°C")
        large_frame.config(bg="yellow")  # 大正方形背景設為黃色
    elif square_id in squares:
        # 更新小正方形的溫度顯示
        color = "red"  # 這裡設定了溫度背景顏色為紅色
        squares[square_id].config(text=f"ID: {square_id}\nTemp: {temperature}°C", bg=color)

def read_uart():
    """
    從 UART 讀取數據，並更新對應的正方形溫度
    """
    global ser
    while True:
        if ser:
            header = ser.read(2)  # 讀取頭部 0xAA, 0x82
            if header == b'\xAA\x82':
                length_byte = ser.read(1)  # 讀取資料長度
                length = struct.unpack('B', length_byte)[0]  # 解包資料長度
                
                if length > 0:
                    data = ser.read(length)  # 根據長度讀取資料內容
                    for i in range(0, len(data), 2):  # 每兩個字節一組 (ID, 溫度)
                        square_id = data[i]
                        temperature = data[i + 1]
                        update_temperature(square_id, temperature)

def on_select_port(value):
    """
    處理選擇串口事件
    :param value: 選擇的串口端口名稱
    """
    selected_port.set(value)
    connect_serial(value)
    print(f"Selected serial port: {value}")

def scan_ports():
    """
    掃描系統可用的串口並更新選單
    """
    ports = [port.device for port in serial.tools.list_ports.comports()]
    for port in ports:
        serial_menu.add_command(label=port, command=lambda p=port: on_select_port(p))
    if not ports:
        serial_menu.add_command(label="No ports available", state=tk.DISABLED)

def send_command():
    """
    每隔一秒發送一次命令
    """
    global ser
    if ser:
        print("Send command")
        ser.write(b'\xAA\x82\x01\x2D')  # 發送命令
    root.after(1000, send_command)  # 每秒調用一次 send_command

# 創建 Tkinter 主窗口
root = tk.Tk()
root.title("九宮格溫度顯示")

# 創建選單
menu_bar = tk.Menu(root)
root.config(menu=menu_bar)
serial_menu = tk.Menu(menu_bar, tearoff=0)
menu_bar.add_cascade(label="Serial Port", menu=serial_menu)

# 創建 StringVar 變量
selected_port = tk.StringVar(value="None")

# 掃描串口並更新選單
scan_ports()

# 創建大正方形的 Frame，ID 為 0
large_frame = tk.Frame(root, width=320, height=320, bg="yellow", borderwidth=2, relief="solid")
large_frame.grid(row=0, column=0, padx=10, pady=10)

# 大正方形顯示 ID 0 和溫度
large_label = tk.Label(large_frame, text="ID: 0\nTemp: N/A", font=("Arial", 16), bg="yellow")
large_label.grid(row=0, column=0, columnspan=3, padx=5, pady=5)

# 儲存所有小正方形 Label
squares = {}

# 創建九個小正方形，排列在大正方形內
for row in range(3):
    for col in range(3):
        small_frame = tk.Frame(large_frame, width=100, height=100, bg="white", borderwidth=1, relief="solid")
        small_frame.grid(row=row + 1, column=col, padx=5, pady=5)
        
        square_id = row * 3 + col + 1  # 小正方形 ID 從 1 開始
        label = tk.Label(small_frame, text=f"ID: {square_id}\nTemp: N/A", font=("Arial", 12), bg="white")
        label.pack(expand=True)
        
        squares[square_id] = label

# 開啟獨立執行緒來讀取 UART 數據
thread = threading.Thread(target=read_uart, daemon=True)
thread.start()

# 每隔一秒發送一次命令
root.after(1000, send_command)

# 開始 Tkinter 主循環
root.mainloop()
