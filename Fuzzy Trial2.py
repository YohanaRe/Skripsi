import random
import time
from ddsm115 import MotorControl
from serial import Serial
print('Logika Fuzzy Sebagai Algoritma pada Sistem Autobrake')


def inference(dist,acci,weight):
  print(f"d: {dist}, a: {acci}, w: {weight}")

  if dist < 6 :
    value_clash = 1
    value_dekat = 0
    value_middle = 0
    value_jauh = 0
    value_clear = 0
  elif dist > 5 and dist < 30 :
    value_clash = 0
    value_dekat = (dist - 5)/(30-5)
    value_middle = 0
    value_jauh = 0
    value_clear = 0
  elif dist == 30 :
    value_clash = 0
    value_dekat = 1
    value_middle = 0
    value_jauh = 0
    value_clear = 0
  elif dist > 30 and dist < 60 :
    value_clash = 0
    value_dekat = (60 - dist)/(60-30)
    value_middle = (dist - 30)/(60-30)
    value_jauh = 0
    value_clear = 0
  elif dist == 60 :
    value_clash = 0
    value_dekat = 0
    value_middle = 1
    value_jauh = 0
    value_clear = 0
  elif dist > 60 and dist < 90 :
    value_clash = 0
    value_dekat = 0
    value_middle = (90 - dist)/(90-60)
    value_jauh = (dist - 60)/(90-60)
    value_clear = 0
  elif dist == 90 :
    value_clash = 0
    value_dekat = 0
    value_middle = 0
    value_jauh = 1
    value_clear = 0
  elif dist > 90 and dist < 120 :
    value_clash = 0
    value_dekat = 0
    value_middle = 0
    value_jauh = (120 - dist)/(120-90)
    value_clear = (dist - 90)/(120-90)
  elif dist > 119 :
    value_clash = 0
    value_dekat = 0
    value_middle = 0
    value_jauh = 0
    value_clear = 1

  # Proses Fuzzifikasi Input Kecepatan
  if acci == 0:
    value_berhentii = 1
    value_lambati = 0
    value_mediumi = 0
    value_cepati = 0
  elif acci > 0 and acci < 0.15:
    value_berhentii = (0.15 - acci) / 0.15
    value_lambati = acci / 0.15
    value_mediumi = 0
    value_cepati = 0
  elif acci == 0.15:
    value_berhentii = 0
    value_lambati = 1
    value_mediumi = 0
    value_cepati = 0
  elif acci > 0.15 and acci < 0.35:
    value_berhentii = 0
    value_lambati = (0.35 - acci) / (0.35 - 0.15)
    value_mediumi = (acci - 0.15) / (0.35 - 0.15)
    value_cepati = 0
  elif acci == 0.35:
    value_berhentii = 0
    value_lambati = 0
    value_mediumi = 1
    value_cepati = 0
  elif acci > 0.35 and acci < 0.5:
    value_berhentii = 0
    value_lambati = 0
    value_mediumi = (0.5 - acci) / (0.5 - 0.35)
    value_cepati = (acci - 0.35) / (0.5 - 0.35)
  elif acci == 0.5:
    value_berhentii = 0
    value_lambati = 0
    value_mediumi = 0
    value_cepati = 1
  elif acci > 0.5 and acci <= 0.6:
    value_berhentii = 0
    value_lambati = 0
    value_mediumi = 0
    value_cepati = (0.6 - acci) / (0.6 - 0.5)
  # else:
  #   value_berhentii = 0
  #   value_lambati = 0
  #   value_mediumi = 0
  #   value_cepati = 0

  #Fuzzifikasi Beban
  if weight >= 0 and weight < 2 :
    value_ringan = 1
    value_sedang = 0
    value_berat = 0
  elif weight == 2 :
    value_ringan = 1
    value_sedang = 0
    value_berat = 0
  elif weight > 2 and weight < 4 :
    value_ringan = (5-weight)/(5-2)
    value_sedang = (weight - 2)/(5-2)
    value_berat = 0
  elif weight == 4 :
    value_ringan = 0
    value_sedang = 1
    value_berat = 0
  elif weight > 4 and weight < 6 :
    value_ringan = 0
    value_sedang = (6-weight)/(6-4)
    value_berat = (weight - 4)/(6-4)
  elif weight == 6 :
    value_ringan = 0
    value_sedang = 0
    value_berat = 1
  elif weight > 6 and weight < 8 :
    value_ringan = 0
    value_sedang = 0
    value_berat = (8-weight)/(8-6)
  elif weight >= 8:
    value_ringan = 0
    value_sedang = 0
    value_berat = 1

  print('Maka Value dari Jarak adalah: ')
  print('Clash  :', value_clash)
  print('Dekat  :', value_dekat)
  print('Middle :', value_middle)
  print('Jauh   :', value_jauh)
  print('Clear  :', value_clear)

  print('Maka Value dari Kecepatan adalah: ')
  print('Berhentii :', value_berhentii)
  print('Lambati   :', value_lambati)
  print('Mediumi   :', value_mediumi)
  print('Cepat     :', value_cepati)

  print('Maka Value dari Beban adalah: ')
  print('Ringan :', value_ringan)
  print('Sedang :', value_sedang)
  print('Berat  :', value_berat)

  #Proses Inferensi
  speed=[]
  def fungsi_berhenti (variabel_jarak, variabel_acci,variabel_berat):
    if variabel_jarak != 0:
      if variabel_acci !=0:
        if variabel_acci !=0:
          hasil_output = min(variabel_jarak, variabel_acci,variabel_berat)
          speed.append([hasil_output,0])

  def fungsi_lambat (variabel_jarak, variabel_acci,variabel_berat):
    if variabel_jarak != 0:
      if variabel_acci !=0:
        if variabel_acci !=0:
          hasil_output = min(variabel_jarak, variabel_acci,variabel_berat)
          speed.append([hasil_output,0.1])

  def fungsi_mid (variabel_jarak, variabel_acci,variabel_berat):
    if variabel_jarak != 0:
      if variabel_acci !=0:
        if variabel_acci !=0:
          hasil_output = min(variabel_jarak, variabel_acci,variabel_berat)
          speed.append([hasil_output,0.25])

  def fungsi_cepat (variabel_jarak, variabel_acci,variabel_berat):
    if variabel_jarak != 0:
      if variabel_acci !=0:
        if variabel_acci !=0:
          hasil_output = min(variabel_jarak, variabel_acci,variabel_berat)
          speed.append([hasil_output,0.43])

  #60 Rules
  fungsi_lambat(value_clear,value_berhentii,value_ringan)
  fungsi_lambat(value_jauh,value_berhentii,value_ringan)
  fungsi_lambat(value_middle,value_berhentii,value_ringan)
  fungsi_lambat(value_dekat,value_berhentii,value_ringan)
  fungsi_berhenti(value_clash,value_berhentii,value_ringan)
  fungsi_cepat(value_clear,value_lambati,value_ringan)
  fungsi_cepat(value_jauh,value_lambati,value_ringan)
  fungsi_mid(value_middle,value_lambati,value_ringan)
  fungsi_lambat(value_dekat,value_lambati,value_ringan)
  fungsi_berhenti(value_clash,value_lambati,value_ringan)
  fungsi_cepat(value_clear,value_mediumi,value_ringan)
  fungsi_cepat(value_jauh,value_mediumi,value_ringan)
  fungsi_mid(value_middle,value_mediumi,value_ringan)
  fungsi_lambat(value_dekat,value_mediumi,value_ringan)
  fungsi_berhenti(value_clash,value_mediumi,value_ringan)
  fungsi_cepat(value_clear,value_cepati,value_ringan)
  fungsi_mid(value_jauh,value_cepati,value_ringan)
  fungsi_mid(value_middle,value_cepati,value_ringan)
  fungsi_lambat(value_dekat,value_cepati,value_ringan)
  fungsi_berhenti(value_clash,value_cepati,value_ringan)
  fungsi_lambat(value_clear,value_berhentii,value_sedang)
  fungsi_lambat(value_jauh,value_berhentii,value_sedang)
  fungsi_lambat(value_middle,value_berhentii,value_sedang)
  fungsi_lambat(value_dekat,value_berhentii,value_sedang)
  fungsi_berhenti(value_clash,value_berhentii,value_sedang)
  fungsi_cepat(value_clear,value_lambati,value_sedang)
  fungsi_cepat(value_jauh,value_lambati,value_sedang)
  fungsi_mid(value_middle,value_lambati,value_sedang)
  fungsi_lambat(value_dekat,value_lambati,value_sedang)
  fungsi_berhenti(value_clash,value_lambati,value_sedang)
  fungsi_cepat(value_clear,value_mediumi,value_sedang)
  fungsi_mid(value_jauh,value_mediumi,value_sedang)
  fungsi_mid(value_middle,value_mediumi,value_sedang)
  fungsi_lambat(value_dekat,value_mediumi,value_sedang)
  fungsi_berhenti(value_clash,value_mediumi,value_sedang)
  fungsi_cepat(value_clear,value_cepati,value_sedang)
  fungsi_mid(value_jauh,value_cepati,value_sedang)
  fungsi_mid(value_middle,value_cepati,value_sedang)
  fungsi_lambat(value_dekat,value_cepati,value_sedang)
  fungsi_berhenti(value_clash,value_cepati,value_sedang)
  fungsi_lambat(value_clear,value_berhentii,value_berat)
  fungsi_lambat(value_jauh,value_berhentii,value_berat)
  fungsi_lambat(value_middle,value_berhentii,value_berat)
  fungsi_lambat(value_dekat,value_berhentii,value_berat)
  fungsi_berhenti(value_clash,value_berhentii,value_berat)
  fungsi_cepat(value_clear,value_lambati,value_berat)
  fungsi_mid(value_jauh,value_lambati,value_berat)
  fungsi_mid(value_middle,value_lambati,value_berat)
  fungsi_lambat(value_dekat,value_lambati,value_berat)
  fungsi_berhenti(value_clash,value_lambati,value_berat)
  fungsi_mid(value_clear,value_mediumi,value_berat)
  fungsi_mid(value_jauh,value_mediumi,value_berat)
  fungsi_mid(value_middle,value_mediumi,value_berat)
  fungsi_lambat(value_dekat,value_mediumi,value_berat)
  fungsi_berhenti(value_clash,value_mediumi,value_berat)
  fungsi_mid(value_clear,value_cepati,value_berat)
  fungsi_mid(value_jauh,value_cepati,value_berat)
  fungsi_lambat(value_middle,value_cepati,value_berat)
  fungsi_lambat(value_dekat,value_cepati,value_berat)
  fungsi_berhenti(value_clash,value_cepati,value_berat)
  #Hasil 
  print('Maka Kecepatan Motor (raw) adalah', speed)
  return speed
  
def get_dist():
  global ser
  dist = [120,120,120]
  while(not ser.readable()):
    print("waiting for serial from dist sensor")
    time.sleep(1)
  while( ser.read().decode("utf-8") is not "*"):
    # print("waiting for ")

    pass
  try:
    data = ser.readline().decode("utf-8")
    print(data)
    data = data.split(",")  
  except:
    print("error reading serial")
    return 120
  if(len(data)<3):
    print("data missing")
    return 120
  dist[0] = int(data[0])
  dist[1] = int(data[1])
  dist[2] = int(data[2])
  return min(dist)

def get_weight():
  global ser2

  while(not ser2.readable()):
    print("waiting for serial2")
    time.sleep(1)
  data =  ser2.readline().decode("utf-8")
  print("weight:", data)
  return int(data)

if __name__ == "__main__":
  ser = Serial("COM3", baudrate=9600) #arduino sensor jarak
  ser2 = Serial("COM7", baudrate=57600) #arduino sensor berat
  motor_kiri = MotorControl("COM6")
  motor_kanan = MotorControl("COM9")
  motor_kiri.set_drive_mode(_id=1, _mode=2)  # Pastikan _id sesuai dengan ID motor Anda
  motor_kanan.set_drive_mode(_id=1, _mode=2)
  motor_kiri.send_rpm(1, 0)
  motor_kanan.send_rpm(1, -(0))
  rpm_kiri, arus_kiri = motor_kiri.get_motor_feedback(1)  
  rpm_kanan, arus_kanan = motor_kanan.get_motor_feedback(1)
  kec = float(input("masukan kecepatan awal (float) :"))
  weight = get_weight()
  dist = get_dist()
  speed = inference(dist,kec,weight)
  num = 0
  den = 0
  print(speed)
  for j in range (0,len(speed)):
    kali = speed[j][0]*speed[j][1]
    bagi = speed[j][0]
    num = num + kali
    den = den + bagi

  # print('Variabel Kali = ', kali)
  # print('Variabel Bagi = ', bagi)
  # print('Variabel num = ', num)
  # print('Variabel den = ', den)
  # if(den !=0):
  xyz = num/den

  print('Maka Kecepatan Motor adalah', xyz, '(m/s)')
  s_rpm = (60/((2*(22/7))* 0.0575))*xyz
  motor_kiri.send_rpm(1, s_rpm)
  motor_kanan.send_rpm(1, -(s_rpm))
  while True:
    dist = get_dist()
    print(f"jarak min : {dist}")
    rpm_kiri, arus_kiri = motor_kiri.get_motor_feedback(1)
    rpm_kanan, arus_kanan = motor_kanan.get_motor_feedback(1)

    speed = inference(dist,((rpm_kiri-rpm_kanan)/2)/(60/((2*(22/7))* 0.0575)),weight)
    num = 0
    den = 0
    for j in range (0,len(speed)):
      kali = speed[j][0]*speed[j][1]
      bagi = speed[j][0]
      num = num + kali
      den = den + bagi

    print('Variabel Kali = ', kali)
    print('Variabel Bagi = ', bagi)
    print('Variabel num = ', num)
    print('Variabel den = ', den)
    # if(den !=0):
    xyz = num/den

    print('Maka Kecepatan Motor adalah', xyz, '(m/s)')
    s_rpm = (60/((2*(22/7))* 0.0575))*xyz
    print('RPM yang dijalankan: ', s_rpm, '(rpm)')  
    motor_kiri.send_rpm(1, s_rpm)
    motor_kanan.send_rpm(1, -(s_rpm))
    if time.time == 30:
      break
time.sleep(0.1)
