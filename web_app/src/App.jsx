import { useEffect, useState, useRef } from "react";
import { Joystick } from "react-joystick-component";
import roslib from "roslib";
import "./App.css";

const CONNECTION_URL = import.meta.env.VITE_CONNECTION_URL || "ws://localhost:9090";
const CMD_VEL_TOPIC = import.meta.env.VITE_CMD_VEL_TOPIC || "/cmd_vel";
const CAMERA_TOPIC = import.meta.env.VITE_CAMERA_TOPIC || "/camera/image/compressed";
const MAX_LIN_VEL = parseFloat(import.meta.env.VITE_MAX_LIN_VEL) || 0.125;
const MAX_ANG_VEL = parseFloat(import.meta.env.VITE_MAX_ANG_VEL) || 1.0;

function App() {
  const [imageSrc, setImageSrc] = useState(null);
  const [cmdVel, setCmdVel] = useState(null);
  const publishInterval = useRef(null);

  useEffect(() => {
    const rosInstance = new roslib.Ros({ url: CONNECTION_URL });

    // Camera topic
    const cameraTopic = new roslib.Topic({
      ros: rosInstance,
      name: CAMERA_TOPIC,
      messageType: "sensor_msgs/msg/CompressedImage",
    });

    cameraTopic.subscribe((message) => {
      setImageSrc(`data:image/jpeg;base64,${message.data}`);
    });

    // Initialize cmd_vel publisher
    const cmdVelTopic = new roslib.Topic({
      ros: rosInstance,
      name: CMD_VEL_TOPIC,
      messageType: "geometry_msgs/msg/Twist",
    });

    setCmdVel(cmdVelTopic);

    return () => {
      cameraTopic.unsubscribe();
      rosInstance.close();
      if (publishInterval.current) {
        clearInterval(publishInterval.current);
      }
    };
  }, []);

  // Function to send velocity commands at a fixed interval
  const handleMove = (event) => {
    if (!cmdVel) return;

    const publishTwist = () => {
      const twist = new roslib.Message({
        linear: { x: event.y * MAX_LIN_VEL, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: -event.x * MAX_ANG_VEL },
      });

      cmdVel.publish(twist);
    };

    // Clear any existing interval before starting a new one
    if (publishInterval.current) {
      clearInterval(publishInterval.current);
    }

    publishTwist();
    publishInterval.current = setInterval(publishTwist, 100);
  };

  // Stop movement when joystick is released
  const handleStop = () => {
    if (!cmdVel) return;

    // Stop publishing
    if (publishInterval.current) {
      clearInterval(publishInterval.current);
      publishInterval.current = null;
    }
  };

  return (
    <div className="main">
      <div className="camera-container">
        <img className="camera" src={imageSrc} alt="Camera video" />
      </div>
      <div className="joystick-container">
        <Joystick
          size={100}
          sticky={false}
          baseColor="#121212"
          stickColor="#ebf20a"
          throttle={100}
          move={handleMove}
          stop={handleStop}
        />
      </div>
    </div>
  );
}

export default App;