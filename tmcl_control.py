#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy

import math
from math import pi
from numpy import *

from serial import Serial
import TMCL
from time import sleep
import time
#from std_msgs.msg import Float64
from geometry_msgs.msg import Twist 
from Motor_steuerung import inverse_kinematic
from winkel_eingabe import Winkel_kardan_1_2
from Dynamically_array import My_list
from reset import home_position

# List of points


motor_12_step = My_list()
motor_13_step = My_list()
motor_14_step = My_list()
motor_15_step = My_list()
motor_16_step = My_list()
motor_17_step = My_list()
x_array = My_list()

serial_port = Serial("/dev/ttyUSB0")    
bus = TMCL.connect(serial_port)

## Motor zuweisung 
motor_17 = bus.get_motor(17)
#time.sleep(0.01)
motor_16 = bus.get_motor(16)
#time.sleep(0.01)   
motor_15 = bus.get_motor(15)
#time.sleep(0.01)
motor_14 = bus.get_motor(14)
time.sleep(0.011)
motor_13 = bus.get_motor(13) 
#time.sleep(0.01)  
motor_12 = bus.get_motor(12)

time_wait=0

#cond1= 0 ; cond2 = 0; cond3 =0 ; cond4 = 0; cond5 = 0; cond6 = 0; cond7 = 0; cond8 = 0


class Tmcl_step_motor_steuerung():
    def __init__(self):
        self.cond1 = 0
        self.cond2 = 0
        self.cond3 = 0
        self.cond4 = 0
        self.cond5 = 0
        self.cond6 = 0
        self.cond7 = 0
        self.cond8 = 0
        self.cond9 = 0
        self.V = 0

        self.time_wait =0

        self.actuel_position_M_16_start = 0
        self.actuel_position_M_15_start = 0
        self.actuel_position_M_14_start = 0
        self.actuel_position_M_13_start = 0    
        self.actuel_position_M_12_start = 0
        
        rospy.loginfo("setting up the node to control the stepper motor")
        rospy.init_node('tmcl_control')

        #--- subscriber to cmd command
        self.ros_sub_twist = rospy.Subscriber("/cmd_vel", Twist, self.get_value_from_keyboard_or_joystick, queue_size=5 )
        rospy.loginfo("subscriber correctly initialozed")
        rospy.spin()

 
    def get_value_from_keyboard_or_joystick(self, message):
        
        rospy.loginfo("angular Components: [%f, %f, %f]"%(message.angular.x, message.angular.y, message.angular.z))
        rospy.loginfo("linear Components: [%f, %f, %f]"%(message.linear.x, message.linear.y, message.linear.z))
        
        

        if message.linear.x != 0 and message.linear.y != 0 and message.linear.z != 0 and message.angular.x != 0 and message.angular.y != 0 and message.angular.z != 0:
            #var = 1


            a = input(
'''
    " Programm durch Ziffer auswählen"

1. Routine erstellen 
2. Routine Beenden und speichern bis das Programm neu gestartet wird
3. Routine laufen lassen
4. Gimbal axe 1, Winkel eingeben
5. Gimbal axe 2, Winkel eingeben
6. Relativ Pose durch x,y,z eingeben
7. Absolute pose durch x,y,z eingeben
8. Null-Position fahren
9. Geschwindigkeit ändern
10. routine durch start und Endpose

Sonst. Robot frei Steuern
'''
)        

            if a ==1:
                    self.cond1 = 1
            elif a ==2:
                    self.cond2 = 2
            elif a ==3:
                    self.cond3 = 3
            elif a ==4:
                    self.cond4 = 4
            elif a ==5:
                    self.cond5 = 5
            elif a ==6:
                    self.cond6 = 6
            elif a ==7:
                    self.cond7 = 7
            elif a ==8:
                    self.cond8 = 8
            elif a ==9:
                    self.cond9 = 9
            else: 
                    print("viel spaß")

            
            ## positionsfeedback // Startpunkt
            if self.cond1 ==1:
                self.actuel_position_M_16_start = motor_16.axis.get(1)  
                self.actuel_position_M_15_start = motor_15.axis.get(1)
                self.actuel_position_M_14_start = motor_14.axis.get(1)  
                self.actuel_position_M_13_start = motor_13.axis.get(1)    
                self.actuel_position_M_12_start = motor_12.axis.get(1)

                # delete Objekt Inhalt
                
                motor_12_step.clear()
                motor_13_step.clear()
                motor_14_step.clear()
                motor_15_step.clear()
                motor_16_step.clear()
                motor_17_step.clear()

                self.cond1 = 2
                print("Teach-in-Modul aktiv")
  
            # Start_punkt:#run_to _last_point()
            if self.cond2 ==2 :
                cond1 =0
                print("Keine Pose mehr wird gespeichert")
                self.cond2 = 13 # != 2

            if self.cond8 == 8 :
                home = home_position()
                self.cond8 == 18

            if self.cond9 == 9 :
                
                G = input('''Geben Sie die neue Geschwindigkeit ein:''')
                if G < 0 :
                    print("Geschwindigkeit muss > 0 sein")

                elif G == 0 or G > 10:
                    print("Geschwindigkeit liegt 0-10 mm/s sein")
                else :
                    self.V = G
                self.cond9 == 19


            if self.cond4 == 4:
                phi_1 = input("geben Sie Phi_1 in grad an:")
                Phi_2 = input("geben Sie Phi_2 in grad an:")

                if phi_1 < 30 and Phi_2 < 30:
                        phi_1 = phi_1*pi/180
                        phi_2 = Phi_2*pi/180
                        Winkel_kardan_1_2(phi_1,Phi_2)
                        phi_1 =0; phi_2 =0
                else:
                        print("Phi_Max  ist 30 grad")
                self.cond4 = 15 #5 # =! 4 
                
            #if self.cond == 5

            if self.cond3 == 3:
                print("routine läuft")

                actuel_position_M_16_end = motor_16.axis.get(1)    
                actuel_position_M_15_end = motor_15.axis.get(1)
                actuel_position_M_14_end = motor_14.axis.get(1)  
                actuel_position_M_13_end = motor_13.axis.get(1)    
                actuel_position_M_12_end = motor_12.axis.get(1)

                Start_punkt_motor_12_step = actuel_position_M_12_end - self.actuel_position_M_12_start
                Start_punkt_motor_13_step = actuel_position_M_13_end - self.actuel_position_M_13_start
                Start_punkt_motor_14_step = actuel_position_M_14_end - self.actuel_position_M_14_start
                Start_punkt_motor_15_step = actuel_position_M_15_end - self.actuel_position_M_15_start
                Start_punkt_motor_16_step = actuel_position_M_16_end - self.actuel_position_M_16_start

                if Start_punkt_motor_12_step !=0 or Start_punkt_motor_13_step != 0 or Start_punkt_motor_14_step != 0:
                    motor_12.axis.set(5,700000)
                    motor_12.move_relative(-1*Start_punkt_motor_12_step)
                    motor_13.axis.set(5,700000)
                    motor_13.move_relative(-1*Start_punkt_motor_13_step)
                    motor_14.axis.set(5,700000)
                    motor_14.move_relative(-1*Start_punkt_motor_14_step)
                    time.sleep(2)

                if Start_punkt_motor_15_step != 0 or Start_punkt_motor_16_step != 0:
                    motor_15.axis.set(5,700000)
                    motor_15.move_relative(-1*Start_punkt_motor_15_step)
                    motor_16.axis.set(5,700000)
                    motor_16.move_relative(-1*Start_punkt_motor_16_step)
                    motor_17.axis.set(5,700000)
                    motor_17.move_relative(-1*Start_punkt_motor_16_step)
                    time.sleep(5)

                if Start_punkt_motor_14_step > Start_punkt_motor_13_step and Start_punkt_motor_14_step > Start_punkt_motor_12_step:
            
                    self.time_wait = Start_punkt_motor_14_step/700000

                elif Start_punkt_motor_13_step > Start_punkt_motor_14_step and Start_punkt_motor_13_step > Start_punkt_motor_12_step:
                
                    self.time_wait = Start_punkt_motor_13_step/700000

                elif Start_punkt_motor_12_step > Start_punkt_motor_14_step and Start_punkt_motor_12_step > Start_punkt_motor_13_step:
                
                    self.time_wait = Start_punkt_motor_12_step/700000

                if self.time_wait < 0:

                    self.time_wait  = self.time_wait * (-1)

                time.sleep( self.time_wait  + 2)

                #for Schleife für die Teach-in 
                i = 0

                #if k == 11:
                #    for 
            
                for i in range (len(motor_12_step)):

                    
                    if not motor_12_step or motor_13_step or motor_14_step or motor_17_step or motor_15_step or motor_16_step:
                        print "Keine Pose wurde gespeichert"
                   
                    #b_Motor_17 = motor_17_step[i]
                    b_Motor_16_17 = motor_16_step[i]
                    b_Motor_15 = motor_15_step[i]
                    b_Motor_14 = motor_14_step[i]
                    b_Motor_13 = motor_13_step[i]
                    b_Motor_12 = motor_12_step[i]
                    x = x_array[i]
                                       
                    v_14 = b_Motor_14
                    v_13 = b_Motor_13
                    v_12 = b_Motor_12

                    v = 5
                    if self.V != 0:
                        v = self.V
                    if v_14 < 0:
                        v_14 = v_14*(-1)

                    if v_13 < 0:
                        v_13 = v_13*(-1)

                    if v_12 < 0:
                        v_12 = v_12*(-1)

                    if v_14 > v_13 and v_14 > v_12:
                        d_per_puls = sqrt(2*x*x - 2*x*x*cos(v/x))
                        v4 = (b_Motor_14*d_per_puls)/1.25
                        v3 = (v4*v_13)/v_14
                        v2 = (v4*v_12)/v_14
                        self.time_wait = v_14/v4

                    if v_13  > v_14 and v_13 > v_12:
                        d_per_puls = sqrt(2*x*x - 2*x*x*cos(v/x))
                        v3 = (b_Motor_13*d_per_puls)/1.25
                        v4 = (v3*v_14)/v_13
                        v2 = (v3*v_12)/v_13

                        self.time_wait = v_13/v3

                    if v_12  > v_14 and v_12 > v_13:
                        d_per_puls = sqrt(2*x*x - 2*x*x*cos(v/x))
                        v3 = (b_Motor_13*d_per_puls)/1.25
                        v4 = (b_Motor_14*d_per_puls)/1.25
                        
                        if v_14 > v_13:
                            
                            v2 = (v_12*v4)/v_14
                        elif v_13 > v_14:
                            v2 = (v_12*v3)/v_13

                        elif v_13 == 0 and v_14 == 0:
                            v3 =0; v4 = 0; v2 = 200000
                         
                            

                        self.time_wait = v_13/v3

                    #print(b_Motor_12,b_Motor_13,b_Motor_14)
                    b_12 = motor_12.axis.get(1)
                    b_13 = motor_13.axis.get(1)
                    b_14 = motor_14.axis.get(1)
                    d_b = 0
                    d_c = 0
                    d_a = 0
                    motor_14.rotate_right(v4)
                    #print(v3)
                    motor_13.rotate_right(v3)
                    motor_12.rotate_right(v2)
                    
                    #motor_15.axis.set(5,400000)
                    #motor_15.move_relative(b_Motor_15)
                
                    #motor_16.axis.set(5,400000)
                    #motor_16.move_relative(b_Motor_16_17)

                    #motor_17.axis.set(5,400000)
                    #motor_17.move_relative(b_Motor_16_17)


                    if v_14 > v_13 and v_14 > v_12:
                        j_4 = b_Motor_14

                        if j_4 < 0:
                            j_4 = j_4 *(-1)

                        while (d_a < j_4):
                            a = motor_14.axis.get(1)
                            d_a = a - b_14
                                
                            if d_a < 0:
                                d_a = d_a * (-1)
                    
                    if v_13 > v_14 and v_13 > v_12:
                        j_3 = b_Motor_13
                        #print("b_Motor_13",b_Motor_13)
                        
                        if j_3 < 0:
                                j_3 = j_3*(-1)
                        
                        while (d_b < j_3):
                            b = motor_13.axis.get(1)
                            d_b = b - b_13
                              
                            if d_b < 0:
                                d_b = d_b*(-1)
                            #print(d_b,j_3)
                            
                        #motor_13.stop()

                    if v_12 > v_14 or v_12 > v_13:
                        j_2 = b_Motor_12

                        if j_2 < 0:
                            j_2 = j_2 * (-1)
                        
                        while (d_c < j_2):
                            c = motor_12.axis.get(1)
                            d_c = c - b_12
                            if d_c < 0:
                                d_c = d_c*(-1)

                    if i == len(motor_12_step) - 1:
                        motor_17.stop()
                        motor_16.stop()
                        motor_15.stop()
                        motor_14.stop()
                        motor_13.stop()
                        motor_12.stop()
                    
                    i+=1
                  

                self.cond3 = 5 # number != 3
            message.linear.x = 0; message.linear.y = 0; message.linear.z = 0; message.angular.x = 0; message.angular.y = 0; message.angular.z= 0

        if message.linear.x != 0 or message.linear.y != 0 or message.linear.z != 0 or message.angular.x != 0 or message.angular.y != 0 or message.angular.z != 0:
            x_inv = 2*message.angular.x
            y_inv = 2*message.angular.y
            z_inv = 2*message.angular.z
            in_rot = message.linear.x
            in_linear = message.linear.y

            value_inv = inverse_kinematic(x_inv, y_inv, z_inv, in_rot, in_linear)

            if self.cond1 ==2:
                motor_12_step.append(value_inv[0])
                motor_13_step.append(value_inv[1])
                motor_14_step.append(value_inv[2])
                motor_15_step.append(value_inv[3])
                motor_16_step.append(value_inv[4])
                motor_17_step.append(value_inv[4])
                x_array.append(value_inv[5])
                #print(motor_12_step)
                #print(motor_12_step)
                #print(motor_14_step)


    def run(self):

        while not rospy.is_shutdown():  
              get_value_from_keyboard_or_joystick()
     

if __name__ == '__main__':
    control_the_motor = Tmcl_step_motor_steuerung()  
    #control_the_motor.run()
