import os
import csv
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, QThread, QCoreApplication, pyqtSignal
from python_qt_binding.QtWidgets import QWidget, QLCDNumber, QTableWidget, QTableWidgetItem, QApplication
from rclpy.qos import QoSProfile
from rqt_gui_py.plugin import Plugin
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.node import Node
import time
import socket
import struct
import signal
import rclpy


class RqtScora(Plugin):
    j5 = 0.0

    def __init__(self, context):
        super(RqtScora, self).__init__(context)
        self.setObjectName('RqtScora')
        self._node = context.node
        self._publisher = None
        self._widget = QWidget()
        _, package_path = get_resource('packages', 'rqt_scora')
        ui_file = os.path.join(package_path, 'share',
                               'rqt_scora', 'resource', 'RqtScora.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RqtScoraUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        # Button clear
        self._widget.homeButton.pressed.connect(self._on_home_btn)
        # Button Grasp
        self._widget.GraspButton.pressed.connect(self._on_grasp_btn)
        # Button Relase
        self._widget.ReleaseButton.pressed.connect(self._on_relase_btn)

        # Slider joint 1
        self._widget.j1_slider.valueChanged.connect(self._on_j1_slider_changed)
        self._widget.j1_slider.setMaximum(7500)
        self._widget.j1_slider.setMinimum(0)
        # Push button increase joint 1
        self._widget.increase_j1_push_button.pressed.connect(
            self._on_strong_increase_j1_pressed)
        # Push button decrease joint 1
        self._widget.decrease_j1_push_button.pressed.connect(
            self._on_strong_decrease_j1_pressed)

        # Slider joint 2
        self._widget.j2_slider.valueChanged.connect(self._on_j2_slider_changed)
        self._widget.j2_slider.setMaximum(10000)
        self._widget.j2_slider.setMinimum(0)
        # Push button increase joint 2
        self._widget.increase_j2_push_button.pressed.connect(
            self._on_strong_increase_j2_pressed)
        # Push button decrease joint 2
        self._widget.decrease_j2_push_button.pressed.connect(
            self._on_strong_decrease_j2_pressed)

        # Slider joint 3
        self._widget.j3_slider.valueChanged.connect(self._on_j3_slider_changed)
        self._widget.j3_slider.setMaximum(3600)
        self._widget.j3_slider.setMinimum(0)
        # Push button increase joint 3
        self._widget.increase_j3_push_button.pressed.connect(
            self._on_strong_increase_j3_pressed)
        # Push button decrease joint 3
        self._widget.decrease_j3_push_button.pressed.connect(
            self._on_strong_decrease_j3_pressed)

        # Slider joint 4
        self._widget.j4_slider.valueChanged.connect(self._on_j4_slider_changed)
        self._widget.j4_slider.setMaximum(3500)
        self._widget.j4_slider.setMinimum(0)
        # Push button increase joint 4
        self._widget.increase_j4_push_button.pressed.connect(
            self._on_strong_increase_j4_pressed)
        # Push button decrease joint 4
        self._widget.decrease_j4_push_button.pressed.connect(
            self._on_strong_decrease_j4_pressed)

        self._widget.SavePosBtn.pressed.connect(self._add_row)

        self._widget.DeletePosBtn.pressed.connect(self._delete_row)

        self._widget.SaveTrayBtn.pressed.connect(self._save_trayectory)

        self._widget.UpTrayBtn.pressed.connect(self._load_trayectory)

        self._widget.ExecutePathButton.pressed.connect(self._execute_rutine)

        self._widget.DeleteRutineButton.pressed.connect(self._delete_rutine)

        self._widget.Connectbtn.pressed.connect(self._connectplc)

        self._widget.disconnectbtn.pressed.connect(self._disconnectplc)

        self._widget.OnButton.pressed.connect(self._on_servo)

        self._widget.OffButton.pressed.connect(self._off_servo)

        self._widget.homeButton.pressed.connect(self._home_arm)

        self._widget.ChgValGeneral.pressed.connect(self._changed_vel_acel_general)

        self._widget.ClcAlarmGeneral.pressed.connect(self._clear_alarm_general)

        self._widget.ChgParamJ1.pressed.connect(self._changed_vel_acel_j1)

        self._widget.ClcAlarmJ1.pressed.connect(self._clear_alarm_j1)

        self._widget.ChgParamJ2.pressed.connect(self._changed_vel_acel_j2)

        self._widget.ClcAlarmJ2.pressed.connect(self._clear_alarm_j2)

        self._widget.ChgParamJ3.pressed.connect(self._changed_vel_acel_j3)

        self._widget.ClcAlarmJ3.pressed.connect(self._clear_alarm_j3)

        self._widget.ChgParamJ4.pressed.connect(self._changed_vel_acel_j4)

        self._widget.ClcAlarmJ4.pressed.connect(self._clear_alarm_j4)


        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.start(100)
        self.zero_cmd_sent = False

        self.state = False

        signal.signal(signal.SIGINT, self.handle_sigint)

        self.username = os.path.expanduser("~")
        self.lista_arq = ''

        if os.path.exists(self.username+"/trajectories_scora"):
            for i in os.listdir(self.username+"/trajectories_scora"):
                self.lista_arq += (i+'\n')
            print(self.lista_arq)
            if len(self.lista_arq) == 0:
                self._widget.ShowText.setText(
                    "No hay rutinas guardadas.\nGuarde las posiciones deseadas.\nAsignele un nombre al archivo.\nProceda a guardar la rutina.")
            else:
                self._widget.ShowText.setText(
                    "Trayectorias guardadas: \n" + self.lista_arq)
        else:
            os.makedirs(self.username+"/trajectories_scora")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.plc_address = ('192.168.1.3', 10000)

        self.updater = update(self.sock, self._widget)

    def handle_sigint(self, signal, frame):
        command = 1
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            self._widget.ShowText1.setText("Desconectado")
            self.updater = update(self.sock, self._widget)
            self.updater.terminate()
            self.updater.wait()
            self.updater.quit()
            self.updater.deleteLater()
        except OSError as e:
            print("Ya te desconectaste")
        self.sock.close()
        print("Cerrando aplicacion")
        QCoreApplication.quit()

    def _connectplc(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.plc_address = ('192.168.1.3', 10000)
        try:
            self.sock.connect(self.plc_address)
            self._widget.ShowText1.setText("Conectado")
            self._widget.ShowText1.setStyleSheet("color: rgb(78, 154, 6);")
            self.state = True
            self.updater = update(self.sock, self._widget)
            self.updater.start_timer()

        except OSError as e:
            print("Cable de red desconectado o PLC apagado")
            self.sock.close()
        except self.sock.error as e:
            print(f'Error: {e}')
            self.sock.close()

    def _disconnectplc(self):
        command = 1
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            print("Desconectado")
            self._widget.ShowText1.setText("Desconectado")
            self._widget.ShowText1.setStyleSheet("color: rgb();")
            self.updater = update(self.sock, self._widget)
            self.updater.terminate()
            self.updater.wait()
            self.updater.quit()
            self.updater.deleteLater()
        except OSError as e:
            print("No estas conectado")
        self.sock.close()

    def _on_servo(self):
        command = 6
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            self._widget.statusj1.setText("Encendido")
            self._widget.statusj2.setText("Encendido")
            self._widget.statusj3.setText("Encendido")
            self._widget.statusj4.setText("Encendido")
        except OSError as e:
            print("No estas conectado")

    def _off_servo(self):
        command = 7
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            self._widget.statusj1.setText("Apagado")
            self._widget.statusj2.setText("Apagado")
            self._widget.statusj3.setText("Apagado")
            self._widget.statusj4.setText("Apagado")
        except OSError as e:
            print("No estas conectado")

    def _home_arm(self):
        command = 10
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
        except OSError as e:
            print("No estas conectado")

        self._widget.ShowText.setText("Posicion inicial")

    def _add_row(self):
        rowcount = self._widget.tablapos.rowCount()
        self._widget.tablapos.insertRow(rowcount)
        j1 = float(self._widget.posencoder1.toPlainText())
        j2 = float(self._widget.posencoder2.toPlainText())
        j3 = float(self._widget.posencoder3.toPlainText())
        j4 = float(self._widget.posencoder4.toPlainText())
        j5 = self.j5
        itemj1 = QTableWidgetItem('{:.2f}'.format(j1))
        itemj2 = QTableWidgetItem('{:.2f}'.format(j2))
        itemj3 = QTableWidgetItem('{:.2f}'.format(j3))
        itemj4 = QTableWidgetItem('{:.2f}'.format(j4))
        itemj5 = QTableWidgetItem('{:.2f}'.format(j5))
        self._widget.tablapos.setItem(rowcount, 0, itemj1)
        self._widget.tablapos.setItem(rowcount, 1, itemj2)
        self._widget.tablapos.setItem(rowcount, 2, itemj3)
        self._widget.tablapos.setItem(rowcount, 3, itemj4)
        self._widget.tablapos.setItem(rowcount, 4, itemj5)

    def _delete_row(self):
        rowcount = self._widget.tablapos.rowCount()
        if rowcount > 0:
            self._widget.tablapos.removeRow(rowcount - 1)

    def _save_trayectory(self):
        name = str(self._widget.NameFileTextEdit.toPlainText())
        locationFile = open(
            self.username+'/trajectories_scora/'+name+'.csv', 'w')
        file = csv.writer(locationFile)
        for row in range(self._widget.tablapos.rowCount()):
            row_data = [self._widget.tablapos.item(
                row, col).text() for col in range(5)]
            file.writerow(row_data)
        self._widget.ShowText.setText(
            "El archivo: "+name+" fue guardado. \nProceda a cargarlo para que pueda ejecutarlo")

    def _load_trayectory(self):
        name = str(self._widget.NameFileTextEdit.toPlainText())
        locationFile = open(
            self.username+'/trajectories_scora/'+name+'.csv', 'r')

        readerfile = csv.reader(locationFile, delimiter=',')
        data = list(readerfile)
        self.row_count = len(data)
        self._widget.tablapos.setRowCount(self.row_count)
        for i in range(self.row_count):
            row_data = data[i]
            for j in range(5):
                item = QTableWidgetItem(row_data[j])
                self._widget.tablapos.setItem(i, j, item)
        self._widget.ShowText.setText(
            "El archivo:  "+name+" fue cargado. \nYa puede ejecutar la rutina.")

    def _delete_rutine(self):
        name = str(self._widget.NameFileTextEdit.toPlainText())
        locationFile = self.username+'/trajectories_scora/'+name+'.csv'
        if os.path.isfile(locationFile):
            os.remove(locationFile)
            self._widget.ShowText.setText("Rutina eliminada")
        else:
            self._widget.ShowText.setText(
                "La rutina no existe, compruebe el nombre del archivo")
        QTimer.singleShot(5000, self.mostrar_rutinas)

    def mostrar_rutinas(self):
        self._widget.ShowText.setText(
            "Trayectorias guardadas: \n" + self.lista_arq)

    def _execute_rutine(self):
        self._widget.ShowText.setText("Ejecutando rutina")
        timeRepeat = self._widget.spinBoxRepeat.value()
        if timeRepeat != 0:
            for i in range(timeRepeat):
                for rows in range(self.row_count):
                    rows_data = []
                    for columns in range(5):
                        cell_value = self._widget.tablapos.item(
                            rows, columns).text()
                        rows_data.append(float(cell_value))
                    self._send_trajectory(
                        rows_data[0], rows_data[1], rows_data[2], rows_data[3], rows_data[4], rows_data[4])

                self._widget.ShowText.setText(
                    "Repeticion #"+str(i+1)+" completada")
            self._widget.ShowText.setText("Rutina completada.")
            self._widget.spinBoxRepeat.setValue(0)
        else:
            for rows in range(self.row_count):
                rows_data = []
                for columns in range(5):
                    cell_value = self._widget.tablapos.item(
                        rows, columns).text()
                    rows_data.append(float(cell_value))

                self._send_trajectory(
                    rows_data[0], rows_data[1], rows_data[2], rows_data[3], rows_data[4], rows_data[4])
            self._widget.ShowText.setText(
                "Rutina completada.\nPuede volver a ejecutar la rutina.")
        QTimer.singleShot(5000, self.mostrar_rutinas)

    def _act_param(self):
        command = 13
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
        data = self.sock.recv(1024)
        fmt = ">f"
        VelocityJoint1 = round(struct.unpack(fmt, data[4:8])[0], 4)
        VelocityJoint2 = round(struct.unpack(fmt, data[14:18])[0], 4)
        VelocityJoint3 = round(struct.unpack(fmt, data[24:28])[0], 4)
        VelocityJoint4 = round(struct.unpack(fmt, data[34:38])[0], 4)
        AcelerationJoint1 = round(struct.unpack(fmt, data[4:8])[0], 4)
        AcelerationJoint2 = round(struct.unpack(fmt, data[14:18])[0], 4)
        AcelerationJoint3 = round(struct.unpack(fmt, data[24:28])[0], 4)
        AcelerationJoint4 = round(struct.unpack(fmt, data[34:38])[0], 4)
        self._widget.VelActualJ1.setText(str(VelocityJoint1))
        self._widget.VelActualJ2.setText(str(VelocityJoint2))
        self._widget.VelActualJ3.setText(str(VelocityJoint3))
        self._widget.VelActualJ4.setText(str(VelocityJoint4))
        self._widget.AcelActualJ1.setText(str(AcelerationJoint1))
        self._widget.AcelActualJ2.setText(str(AcelerationJoint2))
        self._widget.AcelActualJ3.setText(str(AcelerationJoint3))
        self._widget.AcelActualJ4.setText(str(AcelerationJoint4))

    def _on_home_btn(self):
        self._widget.j1_slider.setValue(0)
        self._widget.j2_slider.setValue(0)
        self._widget.j3_slider.setValue(0)
        self._widget.j4_slider.setValue(0)

    def _on_j1_slider_changed(self):
        self._widget.current_j1_label.setText(
            '%0.2f' % (self._widget.j1_slider.value() / 100))
        self._on_parameter_changed()

    def _on_strong_increase_j1_pressed(self):
        self._widget.j1_slider.setValue(
            self._widget.j1_slider.value() + self._widget.j1_slider.pageStep())

    def _on_strong_decrease_j1_pressed(self):
        self._widget.j1_slider.setValue(
            self._widget.j1_slider.value() - self._widget.j1_slider.pageStep())

    def _on_j2_slider_changed(self):
        self._widget.current_j2_label.setText(
            '%0.2f' % (self._widget.j2_slider.value() / 100))
        self._on_parameter_changed()

    def _on_strong_increase_j2_pressed(self):
        self._widget.j2_slider.setValue(
            self._widget.j2_slider.value() + self._widget.j2_slider.pageStep())

    def _on_strong_decrease_j2_pressed(self):
        self._widget.j2_slider.setValue(
            self._widget.j2_slider.value() - self._widget.j2_slider.pageStep())

    def _on_j3_slider_changed(self):
        self._widget.current_j3_label.setText(
            '%0.2f' % (self._widget.j3_slider.value()/100))
        self._on_parameter_changed()

    def _on_strong_increase_j3_pressed(self):
        self._widget.j3_slider.setValue(
            self._widget.j3_slider.value() + self._widget.j3_slider.pageStep())

    def _on_strong_decrease_j3_pressed(self):
        self._widget.j3_slider.setValue(
            self._widget.j3_slider.value() - self._widget.j3_slider.pageStep())

    def _on_j4_slider_changed(self):
        self._widget.current_j4_label.setText(
            '%0.2f' % (self._widget.j4_slider.value()/100))
        self._on_parameter_changed()

    def _on_strong_increase_j4_pressed(self):
        self._widget.j4_slider.setValue(
            self._widget.j4_slider.value() + self._widget.j4_slider.pageStep())

    def _on_strong_decrease_j4_pressed(self):
        self._widget.j4_slider.setValue(
            self._widget.j4_slider.value() - self._widget.j4_slider.pageStep())

    def _on_grasp_btn(self):
        self.j5 = 30.0
        command = 12
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
        self._widget.statusj5.setText("Cerrado")

    def _on_relase_btn(self):
        self.j5 = 0.0
        self._on_parameter_changed()
        command = 11
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
        self._widget.statusj5.setText("Abierto")

    def _on_parameter_changed(self):

        j1 = self._widget.j1_slider.value() / 100
        j2 = self._widget.j2_slider.value() / 100
        j3 = self._widget.j3_slider.value() / 100
        j4 = self._widget.j4_slider.value() / 100
        ji5 = self.j5
        self._send_trajectory(j1, j2, j3, j4, ji5, ji5)

    def _send_trajectory(self, j1, j2, j3, j4, j5, j6):
        value1 = int(j1)
        value2 = int(j2)
        value3 = int(j3)
        value4 = int(j4)
        hex_str1 = format(value1, 'x').zfill(4)
        hex_str2 = format(value2, 'x').zfill(4)
        hex_str3 = format(value3, 'x').zfill(4)
        hex_str4 = format(value4, 'x').zfill(4)
        command = 8
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+hex_str4+hex_str3+hex_str2+hex_str1
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            scora_trajectory = Trajectory_publisher()
            scora_trajectory.goal_positions = [j1, j2, j3, j4, j5, j6]
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = scora_trajectory.joints
            point = JointTrajectoryPoint()
            point.positions = scora_trajectory.goal_positions
            point.time_from_start = Duration(sec=1)
            trajectory_msg.points.append(point)
        except OSError as e:
            print("No estas conectado")

    def _changed_vel_acel_general(self):
        VelGeneral = int(self._widget.VelGeneralValue.toPlainText())
        AcelGeneral = int(self._widget.AcelGeneralValue.toPlainText())
        hex_str1 = format(VelGeneral, 'x').zfill(4)
        hex_str2 = format(AcelGeneral, 'x').zfill(4)
        command = 13
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+hex_str2+hex_str1
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
        except OSError as e:
            print("No estas conectado")
            
    def _clear_alarm_general(self):
        command = 5
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            self._widget.lcdNumber1.display(0)
            self._widget.lcdNumber2.display(0)
            self._widget.lcdNumber3.display(0)
            self._widget.lcdNumber4.display(0)
        except OSError as e:
            print("No estas conectado")

    def _changed_vel_acel_j1(self):
        VelJ1 = int(self._widget.VelJ1Value.toPlainText())
        AcelJ1 = int(self._widget.AcelJ1Value.toPlainText())
        hex_str1 = format(VelJ1, 'x').zfill(4)
        hex_str2 = format(AcelJ1, 'x').zfill(4)
        command = 14
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+hex_str2+hex_str1
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
        except OSError as e:
            print("No estas conectado")
            
    def _clear_alarm_j1(self):
        command = 18
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            self._widget.lcdNumber1.display(0)
        except OSError as e:
            print("No estas conectado")

    def _changed_vel_acel_j2(self):
        VelJ2 = int(self._widget.VelJ2Value.toPlainText())
        AcelJ2 = int(self._widget.AcelJ2Value.toPlainText())
        hex_str1 = format(VelJ2, 'x').zfill(4)
        hex_str2 = format(AcelJ2, 'x').zfill(4)
        command = 15
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+hex_str2+hex_str1
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
        except OSError as e:
            print("No estas conectado")
            
    def _clear_alarm_j2(self):
        command = 19
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            self._widget.lcdNumber2.display(0)
        except OSError as e:
            print("No estas conectado")

    def _changed_vel_acel_j3(self):
        VelJ3 = int(self._widget.VelJ3Value.toPlainText())
        AcelJ3 = int(self._widget.AcelJ3Value.toPlainText())
        hex_str1 = format(VelJ3, 'x').zfill(4)
        hex_str2 = format(AcelJ3, 'x').zfill(4)
        command = 16
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+hex_str2+hex_str1
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
        except OSError as e:
            print("No estas conectado")
            
    def _clear_alarm_j3(self):
        command = 20
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            self._widget.lcdNumber3.display(0)
        except OSError as e:
            print("No estas conectado")

    def _changed_vel_acel_j4(self):
        VelJ4 = int(self._widget.VelJ4Value.toPlainText())
        AcelJ4 = int(self._widget.AcelJ4Value.toPlainText())
        hex_str1 = format(VelJ4, 'x').zfill(4)
        hex_str2 = format(AcelJ4, 'x').zfill(4)
        command = 17
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+hex_str2+hex_str1
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
        except OSError as e:
            print("No estas conectado")
            
    def _clear_alarm_j4(self):
        command = 21
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            self._widget.lcdNumber4.display(0)
        except OSError as e:
            print("No estas conectado")


class Trajectory_publisher(Node):
    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(
            JointTrajectory, publish_topic, 10)
        self.joints = ['link_1_to_base_link', 'link_2_to_link_1', 'link_3_to_link_2',
                       'link_4_to_link_3', 'gripper_left_to_link_4', 'gripper_right_to_link_4']
        self.goal_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class update(QThread):
    def __init__(self, sock, widget):
        super(update, self).__init__()
        self.sock = sock
        self.widget = widget

    def start_timer(self):
        # Crear un QTimer para actualizar el label en intervalos regulares
        self.timer1 = QTimer()
        self.timer1.timeout.connect(self.update_lbl)
        self.timer1.start(500)  # Actualizar cada 1000 ms (1 segundo)

    def update_lbl(self):
        command = 40
        hex_str5 = format(command, 'x').zfill(2)
        value = hex_str5+"0000"+"0000"+"0000"+"0000"
        decimal_num = int(value, 16)
        try:
            self.sock.sendall(decimal_num.to_bytes(9, byteorder='little'))
            data = self.sock.recv(1024)
            fmt = ">f"
            VelocityJoint1 = round(struct.unpack(fmt, data[0:4])[0], 4)
            PositionJoint1 = round(struct.unpack(fmt, data[4:8])[0], 4)
            AccelerationJoint1 = round(struct.unpack(fmt, data[8:12])[0], 4)
            AlarmJoint1 = struct.unpack('h', data[12:14])[0]
            VelocityJoint2 = round(struct.unpack(fmt, data[14:18])[0], 4)
            PositionJoint2 = round(struct.unpack(fmt, data[18:22])[0], 4)
            AccelerationJoint2 = round(struct.unpack(fmt, data[22:26])[0], 4)
            AlarmJoint2 = struct.unpack('h', data[26:28])[0]
            VelocityJoint3 = round(struct.unpack(fmt, data[28:32])[0], 4)
            PositionJoint3 = round(struct.unpack(fmt, data[32:36])[0], 4)
            AccelerationJoint3 = round(struct.unpack(fmt, data[36:40])[0], 4)
            AlarmJoint3 = struct.unpack('h', data[40:42])[0]
            VelocityJoint4 = round(struct.unpack(fmt, data[42:46])[0], 4)
            PositionJoint4 = round(struct.unpack(fmt, data[46:50])[0], 4)
            AccelerationJoint4 = round(struct.unpack(fmt, data[50:54])[0], 4)
            AlarmJoint4 = struct.unpack('h', data[54:56])[0]
            self.widget.posencoder1.setText(str(PositionJoint1))
            self.widget.posencoder2.setText(str(PositionJoint2))
            self.widget.posencoder3.setText(str(PositionJoint3))
            self.widget.posencoder4.setText(str(PositionJoint4))
            self.widget.VelActualJ1.setText(str(VelocityJoint1))
            self.widget.AcelActualJ1.setText(str(AccelerationJoint1))
            self.widget.VelActualJ2.setText(str(VelocityJoint2))
            self.widget.AcelActualJ2.setText(str(AccelerationJoint2))
            self.widget.VelActualJ3.setText(str(VelocityJoint3))
            self.widget.AcelActualJ3.setText(str(AccelerationJoint3))
            self.widget.VelActualJ4.setText(str(VelocityJoint4))
            self.widget.AcelActualJ4.setText(str(AccelerationJoint4))
            self.widget.lcdNumber1.display(AlarmJoint1)
            self.widget.lcdNumber2.display(AlarmJoint2)
            self.widget.lcdNumber3.display(AlarmJoint3)
            self.widget.lcdNumber4.display(AlarmJoint4)
            if (AlarmJoint1 != 0):
                self.widget.lcdNumber1.setStyleSheet("background-color: rgb(239, 41, 41);")
            else:
                self.widget.lcdNumber1.setStyleSheet("background-color: rgb();")

            if (AlarmJoint2 != 0):
                self.widget.lcdNumber2.setStyleSheet("background-color: rgb(239, 41, 41);")
            else:
                self.widget.lcdNumber2.setStyleSheet("background-color: rgb();")

            if (AlarmJoint3 != 0):
                self.widget.lcdNumber3.setStyleSheet("background-color: rgb(239, 41, 41);")
            else:
                self.widget.lcdNumber3.setStyleSheet("background-color: rgb();")

            if (AlarmJoint4 != 0):
                self.widget.lcdNumber4.setStyleSheet("background-color: rgb(239, 41, 41);")
            else:
                self.widget.lcdNumber4.setStyleSheet("background-color: rgb();")

            scora_trajectory = Trajectory_publisher()
            scora_trajectory.goal_positions = [
                PositionJoint1, PositionJoint2, PositionJoint3, PositionJoint4, 0.0, 0.0]
            scora_trajectory.velocities = [
                VelocityJoint1, VelocityJoint2, VelocityJoint3, VelocityJoint4, 0.0, 0.0]
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = scora_trajectory.joints
            point = JointTrajectoryPoint()
            point.positions = scora_trajectory.goal_positions
            point.velocities = scora_trajectory.velocities
            point.time_from_start = Duration(sec=1)
            trajectory_msg.points.append(point)
            scora_trajectory.trajectory_publihser.publish(trajectory_msg)
        except OSError as e:
            print("No estas conectado")
