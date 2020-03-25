import numpy as np
import time
import json
import sys
import requests
from networktables import NetworkTablesInstance

def parseError(str, config_file):
    """Report parse error."""
    print("config error in '" + config_file + "': " + str, file=sys.stderr)


def read_config(config_file):
    """Read configuration file."""
    team = -1

    # parse file
    try:
        with open(config_file, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(config_file, err), file=sys.stderr)
        return team

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object", config_file)
        return team

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number", config_file)

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras", config_file)

    return team

def main(config):
    team = read_config(config)
    WIDTH, HEIGHT = 320, 240

    print("Connecting to Network Tables")
    ntinst = NetworkTablesInstance.getDefault()
    ntinst.startClientTeam(team)

    while True: 
        r = requests.get("https://wonderboxbot.appspot.com/data")
        val = r.json()["command"]
        print("command is {}".format(val))
        if (val != 0):
            """ fill in networktables """
            dat = {"command":0}
            r = requests.post("https://wonderboxbot.appspot.com/data", data = json.dumps(dat))
        time.sleep(0.5)



        
"""
    
    nb_objects_entry = ntinst.getTable("ML").getEntry("nb_objects")
    boxes_entry = ntinst.getTable("ML").getEntry("boxes")
    object_classes_entry = ntinst.getTable("ML").getEntry("object_classes")

    print("Starting camera server")
    cs = CameraServer.getInstance()
    camera = cs.startAutomaticCapture()
    camera.setResolution(WIDTH, HEIGHT)
    cvSink = cs.getVideo()
    img = np.zeros(shape=(HEIGHT, WIDTH, 3), dtype=np.uint8)
    output = cs.putVideo("MLOut", WIDTH, HEIGHT)

    print("Initializing ML engine")
    engine = DetectionEngine("model.tflite")
    parser = PBTXTParser("map.pbtxt")
    parser.parse()
    labels = parser.get_labels()

    start = time()

    print("Starting ML mainloop")
    while True:
        t, frame = cvSink.grabFrame(img)

        # Run inference.
        ans = engine.detect_with_image(Image.fromarray(frame), threshold=0.5, keep_aspect_ratio=True, relative_coord=False, top_k=10)
        nb_objects_entry.setNumber(len(ans))

        boxes = []
        names = []

        # Display result.
        if ans:
            for obj in ans:
                log_object(obj, labels)
                if labels:
                    names.append(labels[obj.label_id])
                box = [round(i, 3) for i in obj.bounding_box.flatten().tolist()]
                boxes.extend(box)
                xmin, ymin, xmax, ymax = map(int,box)

                label = '%s: %d%%' % (names[-1], int(obj.score * 100))  # Example: 'Cargo: 72%'
                label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                label_ymin = max(ymin, label_size[1] + 10)
                cv2.rectangle(frame, (xmin, label_ymin - label_size[1] - 10),
                              (xmin + label_size[0], label_ymin + base_line - 10), (255, 255, 255), cv2.FILLED)
                cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 4)

            output.putFrame(frame)

        else:
            print('No object detected!')
            output.putFrame(img)
        boxes_entry.setDoubleArray(boxes)
        object_classes_entry.setStringArray(names)
        print("FPS: {:.1f}".format(1 / (time() - start)))

        start = time()
"""

if __name__ == '__main__':
    config_file = "/boot/frc.json"
    main(config_file)