import tkinter as tk
from tkinter import ttk
import configparser
import config_data
import serial.tools.list_ports
from tkinter import messagebox
import time
#def apply_settings():

def connect_device():
    print("connect_device")

def update_config() :
# Square 1
    oseeing.set_square_alarm(square1_alarm_var.get(),1)
    print(f"Square1 alarm ={oseeing.square1_alarm}")
    hex_value = square1_alarm_entry.get().strip()
    if int(hex_value) >127 :
        messagebox.showerror("錯誤", "超出範圍(1)，請輸入0~127")
    else :
        oseeing.set_square_alarm_temperature(hex_value, 1)
        print(f"Square1 alarm temperature={oseeing.square1_alarm_temperature}")

# Square 2
    oseeing.set_square_alarm(square1_alarm_var.get(),2)
    print(f"Square2 alarm ={oseeing.square1_alarm}")
    hex_value = square2_alarm_entry.get().strip()
    if int(hex_value) >127 :
        messagebox.showerror("錯誤", "超出範圍(2)，請輸入0~127")
    else :
        oseeing.set_square_alarm_temperature(hex_value, 2)
        print(f"Square2 alarm temperature={oseeing.square2_alarm_temperature}")

# Square 3
    oseeing.set_square_alarm(square3_alarm_var.get(),3)
    print(f"Square3 alarm ={oseeing.square3_alarm}")
    hex_value = square3_alarm_entry.get().strip()
    if int(hex_value) >127 :
        messagebox.showerror("錯誤", "超出範圍(3)，請輸入0~127")
    else :
        oseeing.set_square_alarm_temperature(hex_value, 3)
        print(f"Square3 alarm temperature={oseeing.square3_alarm_temperature}")

# Square 4
    oseeing.set_square_alarm(square4_alarm_var.get(),4)
    print(f"Square4 alarm ={oseeing.square4_alarm}")
    hex_value = square4_alarm_entry.get().strip()
    if int(hex_value) >127 :
        messagebox.showerror("錯誤", "超出範圍(4)，請輸入0~127")
    else :
        oseeing.set_square_alarm_temperature(hex_value, 4)
        print(f"Square4 alarm temperature={oseeing.square1_alarm_temperature}")

# Square 5
    oseeing.set_square_alarm(square5_alarm_var.get(),5)
    print(f"Square5 alarm ={oseeing.square5_alarm}")
    hex_value = square5_alarm_entry.get().strip()
    if int(hex_value) >127 :
        messagebox.showerror("錯誤", "超出範圍(5)，請輸入0~127")
    else :
        oseeing.set_square_alarm_temperature(hex_value, 5)
        print(f"Square5 alarm temperature={oseeing.square5_alarm_temperature}")

# Square 6
    oseeing.set_square_alarm(square6_alarm_var.get(),6)
    print(f"Square6 alarm ={oseeing.square6_alarm}")
    hex_value = square6_alarm_entry.get().strip()
    if int(hex_value) >127 :
        messagebox.showerror("錯誤", "超出範圍(6)，請輸入0~127")
    else :
        oseeing.set_square_alarm_temperature(hex_value, 6)
        print(f"Square6 alarm temperature={oseeing.square6_alarm_temperature}")

# Square 7
    oseeing.set_square_alarm(square7_alarm_var.get(),7)
    print(f"Square7 alarm ={oseeing.square7_alarm}")
    hex_value = square7_alarm_entry.get().strip()
    if int(hex_value) >127 :
        messagebox.showerror("錯誤", "超出範圍(7)，請輸入0~127")
    else :
        oseeing.set_square_alarm_temperature(hex_value, 7)
        print(f"Square7 alarm temperature={oseeing.square7_alarm_temperature}")

# Square 8
    oseeing.set_square_alarm(square8_alarm_var.get(),8)
    print(f"Square8 alarm ={oseeing.square8_alarm}")
    hex_value = square8_alarm_entry.get().strip()
    if int(hex_value) >127 :
        messagebox.showerror("錯誤", "超出範圍(8)，請輸入0~127")
    else :
        oseeing.set_square_alarm_temperature(hex_value, 8)
        print(f"Square8 alarm temperature={oseeing.square8_alarm_temperature}")

# Square 9
    oseeing.set_square_alarm(square1_alarm_var.get(),9)
    print(f"Square9 alarm ={oseeing.square1_alarm}")
    hex_value = square9_alarm_entry.get().strip()
    if int(hex_value) >127 :
        messagebox.showerror("錯誤", "超出範圍(9)，請輸入0~127")
    else :
        oseeing.set_square_alarm_temperature(hex_value, 9)
        print(f"Square9 alarm temperature={oseeing.square9_alarm_temperature}")

# Frame
    oseeing.set_frame_alarm(frame_alarm_var.get())
    print(f"Frame alarm value : {(frame_alarm_var.get())}")
    hex_value = frame_alarm_entry.get().strip()
    if int(hex_value)>127:
        messagebox.showinfo("錯誤", f"輸入的溫度大於於127: {hex_value}")
    else :
        oseeing.set_frame_alarm_temperature(hex_value)
        print(f"Seg Frame alarm temperature = {hex_value}")
    
def parse_default_ini () :
    ini_config = configparser.ConfigParser()
    try :
        ini_config.read('oseeing1s.ini')
    except Exception as e:
        print(f"ERROR : {e}")
    sections = ini_config.sections()
    print('Sections:', sections)
    for section in sections:
        print('Section:', section)
        options = ini_config.options(section)
        if section == "CONFIGURATION" :
            for option in options:
                value = ini_config.get(section, option)
                if option == 'id' :
                    #id = int(value,16)
                    #oseeing.set_id(id)
                    oseeing.set_id(value)
                elif option == 'frame_alarm' :
                    oseeing.set_frame_alarm(value)
                elif option == 'alarm_temperature' :
                    oseeing.set_frame_alarm_temperature(value)
                    print(oseeing.frame_alarm_temperature)
                elif option == 'square1_alarm' :
                    oseeing.set_square_alarm(value,1)
                elif option == 'square2_alarm' :
                    oseeing.set_square_alarm(value,2)
                elif option == 'square3_alarm' :
                    oseeing.set_square_alarm(value,3)
                elif option == 'square4_alarm' :
                    oseeing.set_square_alarm(value,4)
                elif option == 'square5_alarm' :
                    oseeing.set_square_alarm(value,5)
                elif option == 'square6_alarm' :
                    oseeing.set_square_alarm(value,6)
                elif option == 'square7_alarm' :
                    oseeing.set_square_alarm(value,7)
                elif option == 'square8_alarm' :
                    oseeing.set_square_alarm(value,8)
                elif option == 'square9_alarm' :
                    oseeing.set_square_alarm(value,9)
                elif option == 'square1_alarm_temperature' :
                    oseeing.set_square_alarm_temperature(value,1)
                elif option == 'square2_alarm_temperature' :
                    oseeing.set_square_alarm_temperature(value,2)
                elif option == 'square3_alarm_temperature' :
                    oseeing.set_square_alarm_temperature(value,3)
                elif option == 'square4_alarm_temperature' :
                    oseeing.set_square_alarm_temperature(value,4)
                elif option == 'square5_alarm_temperature' :
                    oseeing.set_square_alarm_temperature(value,5)
                elif option == 'square6_alarm_temperature' :
                    oseeing.set_square_alarm_temperature(value,6)
                elif option == 'square7_alarm_temperature' :
                    oseeing.set_square_alarm_temperature(value,7)
                elif option == 'square8_alarm_temperature' :
                    oseeing.set_square_alarm_temperature(value,8)
                elif option == 'square9_alarm_temperature' :
                    oseeing.set_square_alarm_temperature(value,9)

                print(f'{option} = {value}')    

def download_settings() :
    if oseeing.com_port == None :
        messagebox.showerror("錯誤", "未指定COM Port")
        return
 
    #write frame
    update_config()
    ''' if oseeing.set_device_mode() == False :
        messagebox.showerror("錯誤", "Frame 設置異常")
        return
    else :
        print("Set Frame successful...")
        time.sleep(0.01) '''
    if oseeing.frame_alarm == '1':
        sqdata = 0x80 + int(oseeing.frame_alarm_temperature)
        if oseeing.set_square(0,sqdata) == False :
            messagebox.showerror("錯誤", f"Frame 設置異常")
            return
    else :
        if oseeing.set_square(0,0) == False :
            messagebox.showerror("錯誤", f"Frame 設置異常")
            return
    print(f"Set Frame ({sqdata}) successful...")
    time.sleep(0.01)
    
    #square 1
    if oseeing.square1_alarm == '1':
        sqdata = 0x80 + int(oseeing.square1_alarm_temperature)
        if oseeing.set_square(1,sqdata) == False :
            messagebox.showerror("錯誤", f"Square 1 設置異常")
            return
    else :
        if oseeing.set_square(1,0) == False :
            messagebox.showerror("錯誤", f"Square 1 設置異常")
            return
    print(f"Set Square 1 ({sqdata}) successful...")
    time.sleep(0.01)

    #square 2
    if oseeing.square2_alarm == '1':
        sqdata = 0x80 + int(oseeing.square2_alarm_temperature)
        if oseeing.set_square(2,sqdata) == False :
            messagebox.showerror("錯誤", f"Square 2 設置異常")
            return
    else :
        if oseeing.set_square(2,0) == False :
            messagebox.showerror("錯誤", f"Square 2 設置異常")
            return
    print(f"Set Square 2 ({sqdata}) successful...")
    time.sleep(0.01)

    #square 3
    if oseeing.square3_alarm == '1':
        sqdata = 0x80 + int(oseeing.square3_alarm_temperature)
        if oseeing.set_square(3,sqdata) == False :
            messagebox.showerror("錯誤", f"Square 3 設置異常")
            return
    else :
        if oseeing.set_square(3,0) == False :
            messagebox.showerror("錯誤", f"Square 3 設置異常")
            return
    print(f"Set Square 3 ({sqdata}) successful...")
    time.sleep(0.01)

    #square 4
    if oseeing.square4_alarm == '1':
        sqdata = 0x80 + int(oseeing.square4_alarm_temperature)
        if oseeing.set_square(4,sqdata) == False :
            messagebox.showerror("錯誤", f"Square 4 設置異常")
            return
    else :
        if oseeing.set_square(4,0) == False :
            messagebox.showerror("錯誤", f"Square 4 設置異常")
            return
    print(f"Set Square 4 ({sqdata}) successful...")
    time.sleep(0.01)

    #square 5
    if oseeing.square5_alarm == '1':
        sqdata = 0x80 + int(oseeing.square5_alarm_temperature)
        if oseeing.set_square(5,sqdata) == False :
            messagebox.showerror("錯誤", f"Square 5 設置異常")
            return
    else :
        if oseeing.set_square(5,0) == False :
            messagebox.showerror("錯誤", f"Square 5 設置異常")
            return
    print(f"Set Square 5 ({sqdata}) successful...")
    time.sleep(0.01)

    #square 6
    if oseeing.square6_alarm == '1':
        sqdata = 0x80 + int(oseeing.square6_alarm_temperature)
        if oseeing.set_square(6,sqdata) == False :
            messagebox.showerror("錯誤", f"Square 6 設置異常")
            return
    else :
        if oseeing.set_square(6,0) == False :
            messagebox.showerror("錯誤", f"Square 6 設置異常")
            return
    print(f"Set Square 6 ({sqdata}) successful...")
    time.sleep(0.01)

    #square 7
    if oseeing.square7_alarm == '1':
        sqdata = 0x80 + int(oseeing.square7_alarm_temperature)
        if oseeing.set_square(7,sqdata) == False :
            messagebox.showerror("錯誤", f"Square 7 設置異常")
            return
    else :
        if oseeing.set_square(7,0) == False :
            messagebox.showerror("錯誤", f"Square 7 設置異常")
            return
    print(f"Set Square 7 ({sqdata}) successful...")
    time.sleep(0.01)

    #square 8
    if oseeing.square8_alarm == '1':
        sqdata = 0x80 + int(oseeing.square8_alarm_temperature)
        if oseeing.set_square(8,sqdata) == False :
            messagebox.showerror("錯誤", f"Square 8 設置異常")
            return
    else :
        if oseeing.set_square(8,0) == False :
            messagebox.showerror("錯誤", f"Square 8 設置異常")
            return
    print(f"Set Square 8 ({sqdata}) successful...")
    time.sleep(0.01)

    #square 9
    if oseeing.square9_alarm == '1':
        sqdata = 0x80 + int(oseeing.square9_alarm_temperature)
        if oseeing.set_square(9,sqdata) == False :
            messagebox.showerror("錯誤", f"Square 9 設置異常")
            return
    else :
        if oseeing.set_square(9,0) == False :
            messagebox.showerror("錯誤", f"Square 9 設置異常")
            return
    print(f"Set Square 9 ({sqdata}) successful...")
    time.sleep(0.01)

def save_settings():
    print("save settings")
    update_config()
    config = configparser.RawConfigParser()
    with open('oseeing1s.ini', 'w') as file:
        file.write("\n")
    config['CONFIGURATION'] = {
    	'mode': oseeing.mode,
    	'id':oseeing.id,
        'frame_alarm':oseeing.frame_alarm,
        'alarm_temperature':oseeing.frame_alarm_temperature,
        'square1_alarm':oseeing.square1_alarm,
        'square1_alarm_temperature':oseeing.square1_alarm_temperature,
        'square2_alarm':oseeing.square2_alarm,
        'square2_alarm_temperature':oseeing.square2_alarm_temperature,
        'square3_alarm':oseeing.square3_alarm,
        'square3_alarm_temperature':oseeing.square3_alarm_temperature,
        'square4_alarm':oseeing.square4_alarm,
        'square4_alarm_temperature':oseeing.square4_alarm_temperature,
        'square5_alarm':oseeing.square5_alarm,
        'square5_alarm_temperature':oseeing.square5_alarm_temperature,
        'square6_alarm':oseeing.square6_alarm,
        'square6_alarm_temperature':oseeing.square6_alarm_temperature,
        'square7_alarm':oseeing.square7_alarm,
        'square7_alarm_temperature':oseeing.square7_alarm_temperature,
        'square8_alarm':oseeing.square8_alarm,
        'square8_alarm_temperature':oseeing.square8_alarm_temperature,
        'square9_alarm':oseeing.square9_alarm,
        'square9_alarm_temperature':oseeing.square9_alarm_temperature
    	}
    with open('oseeing1s.ini', 'a') as configfile:
        config.write(configfile)
    print("Settings saved to settings.ini")    

def scan_serial_ports():
    ports = serial.tools.list_ports.comports()  # scan serial port
    return [port.device for port in ports]  #  retrun available device

def select_port(selected_port):
    oseeing.set_serial(selected_port)
    com_port_label.config(text=f"COM port device : {selected_port}")
    print(f"Selece COM port: {selected_port}")

def btn_reset_id():
    if oseeing.reset_device_id() :
        messagebox.showerror("確認", f"重置Oseeing裝置ID=AA")
    else :
        messagebox.showerror("錯誤", "連接異常")

def btn_write_id():
    hex_value = write_id_entry.get().strip()
    id = int(hex_value, 16)
    #if len(hex_value) != 2 or not (0x00 <= int(hex_value, 16) <= 0xFF):
    if not (0x00 <= int(hex_value, 16) <= 0xFF):
        messagebox.showerror("錯誤", "請輸入有效的 1-byte 十六進制數（00 - FF）")
        return
    if oseeing.com_port == None :
        messagebox.showerror("錯誤", "未指定COM Port")
        return
    if oseeing.set_device_id(id) :
        messagebox.showerror("確認", f"ID({hex(id)})修改完成")
        oseeing.set_id(hex_value)
        id_entry.delete(0, tk.END)  
        id_entry.insert(0,hex(int(oseeing.id,16))) 

    else :
        messagebox.showerror("錯誤", "連接異常")

def btn_set_id():
    hex_value = id_entry.get().strip()
    id = int(hex_value, 16)
    #if len(hex_value) != 2 or not (0x00 <= int(hex_value, 16) <= 0xFF):
    if not (0x00 <= int(hex_value, 16) <= 0xFF):
        messagebox.showerror("錯誤", "請輸入有效的 1-byte 十六進制數（00 - FF）")
        return
    if oseeing.com_port == None :
        messagebox.showerror("錯誤", "未指定COM Port")
        return
    oseeing.set_id(hex_value)
    if oseeing.set_device_id(id) :
        messagebox.showerror("確認", f"Oseeing 裝置已連接, ID({hex(id)})")
    else :
        messagebox.showerror("錯誤", "連接異常")

# Square 1
def square1_alarm_checkbox_click():
    oseeing.set_square_alarm(square1_alarm_var.get(),1)
    print(f"Square1 alarm ={oseeing.square1_alarm}")
def btn_set_square1_alarm_temperature():
    hex_value = square1_alarm_entry.get().strip()
    if int(hex_value) >127 :
        messagebox.showerror("錯誤", "超出範圍，請輸入0~127")
    else :
        oseeing.set_square_alarm_temperature(hex_value, 1)
        print(f"Square1 alarm temperature={oseeing.square1_alarm_temperature}")

# Square 2
def square2_alarm_checkbox_click():
    oseeing.set_square_alarm(square1_alarm_var.get(),2)
    print(f"Square2 alarm ={oseeing.square1_alarm}")
def btn_set_square2_alarm_temperature():
    hex_value = square2_alarm_entry.get().strip()
    oseeing.set_square_alarm_temperature(hex_value, 2)
    print(f"Square2 alarm temperature={oseeing.square2_alarm_temperature}")

# Square 3
def square3_alarm_checkbox_click():
    oseeing.set_square_alarm(square3_alarm_var.get(),3)
    print(f"Square3 alarm ={oseeing.square3_alarm}")
def btn_set_square3_alarm_temperature():
    hex_value = square3_alarm_entry.get().strip()
    oseeing.set_square_alarm_temperature(hex_value, 3)
    print(f"Square3 alarm temperature={oseeing.square3_alarm_temperature}")

# Square 4
def square4_alarm_checkbox_click():
    oseeing.set_square_alarm(square4_alarm_var.get(),4)
    print(f"Square4 alarm ={oseeing.square4_alarm}")
def btn_set_square4_alarm_temperature():
    hex_value = square4_alarm_entry.get().strip()
    oseeing.set_square_alarm_temperature(hex_value, 4)
    print(f"Square4 alarm temperature={oseeing.square1_alarm_temperature}")

# Square 5
def square5_alarm_checkbox_click():
    oseeing.set_square_alarm(square5_alarm_var.get(),5)
    print(f"Square5 alarm ={oseeing.square5_alarm}")
def btn_set_square5_alarm_temperature():
    hex_value = square5_alarm_entry.get().strip()
    oseeing.set_square_alarm_temperature(hex_value, 5)
    print(f"Square5 alarm temperature={oseeing.square5_alarm_temperature}")

# Square 6
def square6_alarm_checkbox_click():
    oseeing.set_square_alarm(square6_alarm_var.get(),6)
    print(f"Square6 alarm ={oseeing.square6_alarm}")
def btn_set_square6_alarm_temperature():
    hex_value = square6_alarm_entry.get().strip()
    oseeing.set_square_alarm_temperature(hex_value, 6)
    print(f"Square6 alarm temperature={oseeing.square6_alarm_temperature}")

# Square 7
def square7_alarm_checkbox_click():
    oseeing.set_square_alarm(square7_alarm_var.get(),7)
    print(f"Square7 alarm ={oseeing.square7_alarm}")
def btn_set_square7_alarm_temperature():
    hex_value = square7_alarm_entry.get().strip()
    oseeing.set_square_alarm_temperature(hex_value, 7)
    print(f"Square7 alarm temperature={oseeing.square7_alarm_temperature}")

# Square 8
def square8_alarm_checkbox_click():
    oseeing.set_square_alarm(square8_alarm_var.get(),8)
    print(f"Square8 alarm ={oseeing.square8_alarm}")
def btn_set_square8_alarm_temperature():
    hex_value = square8_alarm_entry.get().strip()
    oseeing.set_square_alarm_temperature(hex_value, 8)
    print(f"Square8 alarm temperature={oseeing.square8_alarm_temperature}")

# Square 9
def square9_alarm_checkbox_click():
    oseeing.set_square_alarm(square1_alarm_var.get(),9)
    print(f"Square9 alarm ={oseeing.square1_alarm}")
def btn_set_square9_alarm_temperature():
    hex_value = square9_alarm_entry.get().strip()
    oseeing.set_square_alarm_temperature(hex_value, 9)
    print(f"Square9 alarm temperature={oseeing.square9_alarm_temperature}")


def frame_alarm_checkbox_click():
    #eth.set_transfer_format(rj45_data_format.get())
    oseeing.set_frame_alarm(frame_alarm_var.get())
    print(f"Frame alarm value : {(frame_alarm_var.get())}")

def btn_set_frame_alarm_temperature():
    hex_value = frame_alarm_entry.get().strip()
    if int(hex_value)>127:
        messagebox.showinfo("錯誤", f"輸入的溫度大於於127: {hex_value}")
    else :
        oseeing.set_frame_alarm_temperature(hex_value)
        print(f"Seg Frame alarm temperature = {hex_value}")

oseeing=config_data.SerialConfig()
serial_ports = scan_serial_ports()
global mac_result
global ip_reslut 
ip_result= "None"
mac_result = "None"

print(serial_ports)
parse_default_ini()
#print_ini_setting()
# 建立主視窗
root = tk.Tk()
root.title("Settings")
root.geometry('800x600')
root.resizable(True,True)

# 建立 COM Port 標籤和文字輸入欄
menu = tk.Menu(root)
serial_menu = tk.Menu(menu, tearoff=0)
for port in serial_ports:
    serial_menu.add_command(label=port, command=lambda p=port: select_port(p))
    
menu.add_cascade(label="Select Serial Ports Device Name.....", menu=serial_menu)
root.config(menu=menu)
 
com_port_label = tk.Label(root, text="COM Port:")
com_port_label.grid(row=0,column=0,sticky=tk.W)
# Set Modbus ID

rs485_id_label = tk.Label(root, text="Modbus Device ID:")
rs485_id_label.grid(row=1,column=0,padx=5, pady=5)
id_entry = tk.Entry(root, width=5)
#id_entry.insert(0,hex(oseeing.id).upper()) 
id_entry.insert(0,hex(int(oseeing.id,16))) 
id_entry.grid(row=1, column=1, padx=5)
button_id = tk.Button(root, text="連線", command=btn_set_id)
button_id.grid(row=1, column=2, padx=5)

# Set Frame Alarm params
frame_alarm_var=tk.StringVar(value=oseeing.frame_alarm)
frame_alarm_checkbox = tk.Checkbutton(root, text="Frame Alarm", variable=frame_alarm_var,onvalue=1, offvalue=0,command=frame_alarm_checkbox_click)
frame_alarm_checkbox.grid(row=2,column=0,padx=5,sticky='w')
frame_alarm_temperature = tk.Label(root, text="Frame Alarm Temperature:")
frame_alarm_entry = tk.Entry(root, width=5)
frame_alarm_entry.insert(0,oseeing.frame_alarm_temperature)
frame_alarm_entry.grid(row=2, column=1, padx=5)
temp_frame = tk.Label(root, text="°C")
temp_frame.grid(row=2, column=2, padx=0)
#button_frame_alarm = tk.Button(root, text="確認", command=btn_set_frame_alarm_temperature)
#button_frame_alarm.grid(row=2, column=3, padx=5)

# Set Square1 Alarm params
square1_alarm_var=tk.StringVar(value=oseeing.square1_alarm)
square1_alarm_checkbox = tk.Checkbutton(root, text="Square1 Alarm", variable=square1_alarm_var,onvalue=1, offvalue=0,command=square1_alarm_checkbox_click)
square1_alarm_checkbox.grid(row=3,column=0,padx=5,sticky='w')
square1_alarm_temperature = tk.Label(root, text="Frame Alarm Temperature:")
square1_alarm_entry = tk.Entry(root, width=5)
square1_alarm_entry.insert(0,oseeing.square1_alarm_temperature)
square1_alarm_entry.grid(row=3, column=1, padx=5)
temp_unit_label = tk.Label(root, text="°C")
temp_unit_label.grid(row=3, column=2, padx=0)
#button_square1 = tk.Button(root, text="確認", command=btn_set_square1_alarm_temperature)
#button_square1.grid(row=3, column=3, padx=5)

# Set Square2 Alarm params
square2_alarm_var=tk.StringVar(value=oseeing.square2_alarm)
square2_alarm_checkbox = tk.Checkbutton(root, text="square2 Alarm", variable=square2_alarm_var,onvalue=1, offvalue=0,command=square2_alarm_checkbox_click)
square2_alarm_checkbox.grid(row=4,column=0,padx=5,sticky='w')
square2_alarm_temperature = tk.Label(root, text="Frame Alarm Temperature:")
square2_alarm_entry = tk.Entry(root, width=5)
square2_alarm_entry.insert(0,oseeing.square2_alarm_temperature)
square2_alarm_entry.grid(row=4, column=1, padx=5)
temp_unit_label = tk.Label(root, text="°C")
temp_unit_label.grid(row=4, column=2, padx=0)
#button_square2 = tk.Button(root, text="確認", command=btn_set_square2_alarm_temperature)
#button_square2.grid(row=4, column=3, padx=5)

# Set Square3 Alarm params
square3_alarm_var=tk.StringVar(value=oseeing.square3_alarm)
square3_alarm_checkbox = tk.Checkbutton(root, text="square3 Alarm", variable=square3_alarm_var,onvalue=1, offvalue=0,command=square3_alarm_checkbox_click)
square3_alarm_checkbox.grid(row=5,column=0,padx=5,sticky='w')
square3_alarm_temperature = tk.Label(root, text="Frame Alarm Temperature:")
square3_alarm_entry = tk.Entry(root, width=5)
square3_alarm_entry.insert(0,oseeing.square3_alarm_temperature)
square3_alarm_entry.grid(row=5, column=1, padx=5)
temp_unit_label = tk.Label(root, text="°C")
temp_unit_label.grid(row=5, column=2, padx=0)
#button_square3 = tk.Button(root, text="確認", command=btn_set_square3_alarm_temperature)
#button_square3.grid(row=5, column=3, padx=5)

# Set Square4 Alarm params
square4_alarm_var=tk.StringVar(value=oseeing.square4_alarm)
square4_alarm_checkbox = tk.Checkbutton(root, text="square4 Alarm", variable=square4_alarm_var,onvalue=1, offvalue=0,command=square4_alarm_checkbox_click)
square4_alarm_checkbox.grid(row=6,column=0,padx=5,sticky='w')
square4_alarm_temperature = tk.Label(root, text="Frame Alarm Temperature:")
square4_alarm_entry = tk.Entry(root, width=5)
square4_alarm_entry.insert(0,oseeing.square4_alarm_temperature)
square4_alarm_entry.grid(row=6, column=1, padx=5)
temp_unit_label = tk.Label(root, text="°C")
temp_unit_label.grid(row=6, column=2, padx=0)
#button_square4 = tk.Button(root, text="確認", command=btn_set_square4_alarm_temperature)
#button_square4.grid(row=6, column=3, padx=5)

# Set Square5 Alarm params
square5_alarm_var=tk.StringVar(value=oseeing.square5_alarm)
square5_alarm_checkbox = tk.Checkbutton(root, text="square5 Alarm", variable=square5_alarm_var,onvalue=1, offvalue=0,command=square5_alarm_checkbox_click)
square5_alarm_checkbox.grid(row=7,column=0,padx=5,sticky='w')
square5_alarm_temperature = tk.Label(root, text="Frame Alarm Temperature:")
square5_alarm_entry = tk.Entry(root, width=5)
square5_alarm_entry.insert(0,oseeing.square5_alarm_temperature)
square5_alarm_entry.grid(row=7, column=1, padx=5)
temp_unit_label = tk.Label(root, text="°C")
temp_unit_label.grid(row=7, column=2, padx=0)
#button_square5 = tk.Button(root, text="確認", command=btn_set_square5_alarm_temperature)
#button_square5.grid(row=7, column=3, padx=5)

# Set Square6 Alarm params
square6_alarm_var=tk.StringVar(value=oseeing.square6_alarm)
square6_alarm_checkbox = tk.Checkbutton(root, text="square6 Alarm", variable=square6_alarm_var,onvalue=1, offvalue=0,command=square6_alarm_checkbox_click)
square6_alarm_checkbox.grid(row=8,column=0,padx=5,sticky='w')
square6_alarm_temperature = tk.Label(root, text="Frame Alarm Temperature:")
square6_alarm_entry = tk.Entry(root, width=5)
square6_alarm_entry.insert(0,oseeing.square6_alarm_temperature)
square6_alarm_entry.grid(row=8, column=1, padx=5)
temp_unit_label = tk.Label(root, text="°C")
temp_unit_label.grid(row=8, column=2, padx=0)
#button_square6 = tk.Button(root, text="確認", command=btn_set_square6_alarm_temperature)
#button_square6.grid(row=8, column=3, padx=5)

# Set Square7 Alarm params
square7_alarm_var=tk.StringVar(value=oseeing.square7_alarm)
square7_alarm_checkbox = tk.Checkbutton(root, text="square7 Alarm", variable=square7_alarm_var,onvalue=1, offvalue=0,command=square7_alarm_checkbox_click)
square7_alarm_checkbox.grid(row=9,column=0,padx=5,sticky='w')
square7_alarm_temperature = tk.Label(root, text="Frame Alarm Temperature:")
square7_alarm_entry = tk.Entry(root, width=5)
square7_alarm_entry.insert(0,oseeing.square7_alarm_temperature)
square7_alarm_entry.grid(row=9, column=1, padx=5)
temp_unit_label = tk.Label(root, text="°C")
temp_unit_label.grid(row=9, column=2, padx=0)
#button_square7 = tk.Button(root, text="確認", command=btn_set_square7_alarm_temperature)
#button_square7.grid(row=9, column=3, padx=5)

# Set Square8 Alarm params
square8_alarm_var=tk.StringVar(value=oseeing.square8_alarm)
square8_alarm_checkbox = tk.Checkbutton(root, text="square8 Alarm", variable=square8_alarm_var,onvalue=1, offvalue=0,command=square8_alarm_checkbox_click)
square8_alarm_checkbox.grid(row=10,column=0,padx=5,sticky='w')
square8_alarm_temperature = tk.Label(root, text="Frame Alarm Temperature:")
square8_alarm_entry = tk.Entry(root, width=5)
square8_alarm_entry.insert(0,oseeing.square8_alarm_temperature)
square8_alarm_entry.grid(row=10, column=1, padx=5)
temp_unit_label = tk.Label(root, text="°C")
temp_unit_label.grid(row=10, column=2, padx=0)
#button_square8 = tk.Button(root, text="確認", command=btn_set_square8_alarm_temperature)
#button_square8.grid(row=10, column=3, padx=5)

# Set Square9 Alarm params
square9_alarm_var=tk.StringVar(value=oseeing.square9_alarm)
square9_alarm_checkbox = tk.Checkbutton(root, text="square9 Alarm", variable=square9_alarm_var,onvalue=1, offvalue=0,command=square9_alarm_checkbox_click)
square9_alarm_checkbox.grid(row=11,column=0,padx=5,sticky='w')
square9_alarm_temperature = tk.Label(root, text="Frame Alarm Temperature:")
square9_alarm_entry = tk.Entry(root, width=5)
square9_alarm_entry.insert(0,oseeing.square9_alarm_temperature)
square9_alarm_entry.grid(row=11, column=1, padx=5)
temp_unit_label = tk.Label(root, text="°C")
temp_unit_label.grid(row=11, column=2, padx=0)
#button_square9 = tk.Button(root, text="確認", command=btn_set_square9_alarm_temperature)
#button_square9.grid(row=11, column=3, padx=5)

save_button = tk.Button(root, text="儲存設定", command=save_settings)
save_button.grid(row=12,column=0)

write_button = tk.Button(root, text="寫入裝置", command=download_settings)
write_button.grid(row=12,column=1)


write_id_label = tk.Label(root, text="New Device ID:")
write_id_label.grid(row=13,column=0,padx=5, pady=5)
write_id_entry = tk.Entry(root, width=5)
#id_entry.insert(0,(oseeing.id))
write_id_entry.grid(row=13, column=1, padx=5)
button_write_id = tk.Button(root, text="確認", command=btn_write_id)
button_write_id.grid(row=13, column=2, padx=5)
button_reset_id = tk.Button(root, text="重置ID", command=btn_reset_id)
button_reset_id.grid(row=13, column=3, padx=5)

# 啟動主迴圈
root.mainloop()

