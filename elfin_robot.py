import time
import numpy as np
import threading

import elfin

class elfin_server():
    def __init__(self, server_ip, port_number):
        self.server_ip = server_ip
        self.port_number = port_number
        #print(cobot.ReadPcsActualPos())

    def Initialize(self):
        SIZE = 1024
        rbtID = 0
        self.cobot = elfin.elfin()
        self.cobot.connect(self.server_ip, self.port_number, SIZE, rbtID)
        print("conected!")

    def Run(self):
        #target = [540.0, -30.0, 850.0, 140.0, -81.0, -150.0]
        #print("starting move")
        return self.cobot.ReadPcsActualPos()

    def SendCoordinates(self, target):
        status = self.cobot.ReadMoveState()
        if status != 1009:
            self.cobot.MoveL(target)

    def SendCoordinatesArc(self, target_linear, target_arc):
        status = self.cobot.ReadMoveState()


        print(self.cobot.MoveL(target_linear))
        status = self.cobot.ReadMoveState()
        while status == 1009:
            time.sleep(0.5)
            print("moving...")
            print(self.cobot.ReadPcsActualPos())
            status = self.cobot.ReadMoveState()
        print("end move")

        print(self.cobot.MoveC(target_arc))
        status = self.cobot.ReadMoveState()
        while status == 1009:
            time.sleep(0.5)
            print("moving...")
            print(self.cobot.ReadPcsActualPos())
            status = self.cobot.ReadMoveState()
        print("end move")


    def StopMove(self):
        self.cobot.GrpStop()
        time.sleep(0.1)


    def Close(self):
        self.cobot.close()

#TODO:SendCoordinates2Robot Thread
class SendCoordinates2Robot(threading.Thread):
    def __init__(self, sendcoord_queue, event, sle):
        """Class (threading) to compute real time tractography data for visualization in a single loop.


        Sleep function in run method is used to avoid blocking GUI and more fluent, real-time navigation

        :param inp: List of inputs: trekker instance, affine numpy array, seed_offset, seed_radius, n_threads
        :type inp: list
        :param affine_vtk: Affine matrix in vtkMatrix4x4 instance to update objects position in 3D scene
        :type affine_vtk: vtkMatrix4x4
        :param coord_queue: Queue instance that manage coordinates read from tracking device and coregistered
        :type coord_queue: queue.Queue
        :param visualization_queue: Queue instance that manage coordinates to be visualized
        :type visualization_queue: queue.Queue
        :param event: Threading event to coordinate when tasks as done and allow UI release
        :type event: threading.Event
        :param sle: Sleep pause in seconds
        :type sle: float
        """

        threading.Thread.__init__(self, name='Send Coord to Robot')

        self.target = None
        # self.__bind_events()

        # self.coord_queue = coord_queue
        self.sendcoord_queue = sendcoord_queue
        self.event = event
        self.sle = sle

    def run(self):

        while not self.event.is_set():
            trigger_on = False
            try:
                self.trigger_init.write(b'0')
                sleep(0.3)
                lines = self.trigger_init.readlines()
                # Following lines can simulate a trigger in 3 sec repetitions
                # sleep(3)
                # lines = True
                if lines:
                    trigger_on = True
                    # wx.CallAfter(Publisher.sendMessage, 'Create marker')

                if self.stylusplh:
                    trigger_on = True
                    # wx.CallAfter(Publisher.sendMessage, 'Create marker')
                    self.stylusplh = False

                self.trigger_queue.put_nowait(trigger_on)
                sleep(self.sle)

            except:
                print("Trigger not read, error")
                pass

            # except queue.Empty:
            #     pass
            # except queue.Full:
            #     self.coord_queue.task_done()
        else:
            if self.trigger_init:
                self.trigger_init.close()
