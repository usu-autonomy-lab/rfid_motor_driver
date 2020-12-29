#!/usr/bin/python
import Tkinter as tk
from Tkinter import * 
import tkMessageBox as messagebox
# from PIL import ImageTk, Image
import pygubu
from time import sleep
import os

file_path = os.path.dirname(os.path.abspath(__file__))
UI = os.path.join(file_path, 'bench_gui.ui')

class Application:
    def __init__(self,master):
        self.builder=builder=pygubu.Builder()
        builder.add_from_file(UI)
        self.mainwindow=builder.get_object('bench_gui',master)
        builder.connect_callbacks(self)
        bench_gui.protocol("WM_DELETE_WINDOW", self.on_closing)

#  attempt to add background image      
#        cover=ImageTk.PhotoImage(Image.open("home.png")) 
#        lb_cover=Label(self.mainwindow,image=cover)
#        lb_cover.image=cover
#        lb_cover.grid(column=0,row=0)

    def click_start(self):
        global bench_gui, params
        iterations = float(self.builder.get_object('Iterations_box').get())
        acceleration = float(self.builder.get_object('acceleration_box').get())
        deceleration = float(self.builder.get_object('deceleration_box').get())
        speed = float(self.builder.get_object('Speed_box').get())
        current_limit = float(self.builder.get_object('current_limit_box').get())
        bench_x = float(self.builder.get_object('bench_x_box').get())
        bench_y = float(self.builder.get_object('bench_y_box').get())
        bench_z = float(self.builder.get_object('bench_z_box').get())
        bench_angle = float(self.builder.get_object('bench_angle_box').get())
        rate = float(self.builder.get_object('publish_rate_box').get())
        params = [iterations, acceleration, deceleration, speed,
             current_limit, bench_x, bench_y, bench_z, bench_angle, rate]
        bench_gui.quit()
        bench_gui.destroy()

    def on_closing(self):
        if messagebox.askyesno("Quit", "Quit?"):
            bench_gui.quit()
            bench_gui.destroy()

def gui():
    global bench_gui, params
    bench_gui=tk.Tk()
    bench_gui.title("Bench Test")

    app=Application(bench_gui)
    bench_gui.mainloop()
    return params