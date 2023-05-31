from selenium import webdriver
from selenium.webdriver.chrome.service import Service
from webdriver_manager.chrome import ChromeDriverManager
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.common.by import By
from selenium.webdriver.support.wait import WebDriverWait

import time
js1 = '''
cam = document.querySelector("#app > div > section.maincontent.column > div.videoBlocks > div > div.canvasContainer > canvas")

function record(canvas, time) {
    var recordedChunks = [];
    return new Promise(function (res, rej) {
        var stream = canvas.captureStream(20 /*fps*/);
        mediaRecorder = new MediaRecorder(stream, {
            mimeType: "video/webm; codecs=vp9"
        });
        
        //ondataavailable will fire in interval of `time || 4000 ms`
        mediaRecorder.start(time || 4000);

        mediaRecorder.ondataavailable = function (event) {
            recordedChunks.push(event.data);
             // after stop `dataavilable` event run one more time
            if (mediaRecorder.state === 'recording') {
                mediaRecorder.stop();
            }

        }

        mediaRecorder.onstop = function (event) {
            var blob = new Blob(recordedChunks, {type: "video/webm" });
            var url = URL.createObjectURL(blob);
            res(url);
        }
    })
}

const recording = record(cam,''' 

js2 = ''')
// play it on another video element
var video$ = document.createElement('video')
document.body.appendChild(video$)
recording.then(url => video$.setAttribute('src', url) )

// download it
var link$ = document.createElement('a')
link$.setAttribute('download','''

js3 = ''') 
recording.then(url => {
 link$.setAttribute('href', url) 
 link$.click()
})
'''

webip = 'http://192.168.12.1/'
cams = ['cam1', 'cam2', 'cam3', 'cam4', 'cam5']
rec_time = 20

driver = webdriver.Chrome(service=Service(ChromeDriverManager().install()))

tabs = dict()

def construct_script(name, time):
	return js1 + str(int(time * 1000)) + js2 + "'" + str(name) + "'" + js3

def openCam(cam: str):
	if len(tabs) != 0:
		driver.switch_to.new_window('tab')
	tabs[cam] = driver.current_window_handle
	driver.get(webip + "visiontest")
	driver.find_element(By.CSS_SELECTOR, "#app > div > section.maincontent.column > div.videoSelectors > div > i").click()
	camsel = driver.find_element(By.CSS_SELECTOR, "#app > div > section.maincontent.column > div.videoSelectors > div.selector > input.ext")
	camsel.send_keys("/"+cam)
	camsel.send_keys(Keys.ENTER)
	
def recordCam(cam: str, js_script: str):
	driver.switch_to.window(tabs[cam])
	driver.execute_script(js_script)


for cam in cams:
	openCam(cam)

rec_time = input("Ready, rec how many secs?\n")

for cam in cams:
	recordCam(cam, construct_script(cam + '_' + time.strftime("%H:%M:%S", time.localtime()), int(rec_time)))

for i in range(0, int(rec_time), 1):
	print(f"Started recording for {i}/{rec_time} seconds", end='\r')
	time.sleep(1)

input("\nPress enter to exit")
