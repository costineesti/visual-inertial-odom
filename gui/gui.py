import tkinter as tk


class GUI:
    def __init__(self, fullscreen=True):
        # create the main window
        self.root = tk.Tk()
        self.root.title("DEMO")

        # set font size
        self.fontSize = 16

        self.takePhotosVar = tk.BooleanVar()
        self.keepOldPhotosVar = tk.BooleanVar()

        self.startLive = False
        self.startVideo = False
        self.startCalibrate = False
        self.takePhotos = False
        self.keepOldPhotos = False

        self.createWidgets()
        self.configureWidgets()

        if fullscreen:
            self.root.attributes("-fullscreen", True)

        self.root.mainloop()

    def createWidgets(self):
        # left side widgets
        demoLabel = tk.Label(self.root, text="Demo", font=("Helvetica", self.fontSize))
        demoLabel.grid(row=1, column=0, padx=50, pady=15)

        # live button
        liveButton = tk.Button(self.root, text="Live", command=self.live, width=15, height=3, font=("Helvetica", self.fontSize))
        liveButton.grid(row=2, column=0, padx=50, pady=5, sticky="nesw")

        # video button
        videoButton = tk.Button(self.root, text="Video", command=self.video, width=15, height=3, font=("Helvetica", self.fontSize))
        videoButton.grid(row=3, column=0, padx=50, pady=5, sticky="nesw")

        # right side widgets
        calibrationLabel = tk.Label(self.root, text="Calibration", font=("Helvetica", self.fontSize))
        calibrationLabel.grid(row=1, column=1, padx=50, pady=15)

        # take images
        takePhotosCheckbox = tk.Checkbutton(self.root, text="Take Photos", variable=self.takePhotosVar, font=("Helvetica", self.fontSize))
        takePhotosCheckbox.grid(row=2, column=1, padx=50, pady=5)

        # keep old images
        keepOldPhotosCheckbox = tk.Checkbutton(self.root, text="Keep Old Photos", variable=self.keepOldPhotosVar, font=("Helvetica", self.fontSize))
        keepOldPhotosCheckbox.grid(row=3, column=1, padx=50, pady=5)

        # start button
        startButton = tk.Button(self.root, text="Start Calibration", command=self.calibrate, width=15, height=3, font=("Helvetica", self.fontSize))
        startButton.grid(row=4, column=1, columnspan=2, padx=50, pady=5, sticky="nesw")

        # quit button
        quitButton = tk.Button(self.root, text="Quit", command=self.quit, width=15, height=3, font=("Helvetica", self.fontSize))
        quitButton.grid(row=5, column=0, columnspan=2, padx=50, pady=30, sticky="nesw")

        line = tk.Canvas(self.root, width=2, height=2, bg="black")
        line.grid(row=0, column=0, rowspan=5, columnspan=2, pady=10, sticky="ns")

    def configureWidgets(self):
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)
        self.root.rowconfigure(2, weight=1)
        self.root.rowconfigure(3, weight=1)
        self.root.rowconfigure(4, weight=1)

    def live(self):
        self.startLive = True
        self.root.destroy()

    def video(self):
        self.startVideo = True
        self.root.destroy()

    def calibrate(self):
        self.startCalibrate = True
        self.takePhotos = self.takePhotosVar.get()
        self.keepOldPhotos = self.keepOldPhotosVar.get() 
        self.root.destroy()

    def quit(self):
        self.root.destroy()
        exit()


if __name__ == "__main__": 
    gui = GUI(fullscreen=False)
