
https://github.com/user-attachments/assets/ceef3a35-c54f-4f62-bfaf-e326d8fde0ad
## Autonomous Racing Car (BR-88)
This is hobby project where I built an RC car from scratch to train it to race autonomously in predetermined tracks. In case you are wondering what BR-88 stand for, it is Blind Racer 88. The cars chassis and steering system is built in DIY style. It is a rear wheel drive car. It uses two DC motors for throttle in the rear and one servo motor for steering. \n\n
For racing autonomously, it leverages the values from 3 distance sensors(VL530Lx ToF sensors)  and uses a neural network to predict an action for throttle and steering. The car has been modelled in Blender and Unity to accurately replicate real world physics. WheelColliders are used for the replicating the car physics.

### Training
The car is trained using Reinforcement Learning (Proximal Policy Optimization). The neural network has 2 hiden layers with 128 neurons each. The best model is trained for 32500000 steps.


https://github.com/user-attachments/assets/17f142e5-821e-44fd-89d5-e9550f1c0187


### Circuit Diagram 
![circuit_diagram](https://github.com/user-attachments/assets/4a68c3bb-9bb9-40c6-b3c8-c72cbd81b17a)

### Pictures

![WhatsApp Image 2024-05-12 at 12 07 00_2a5a48d1](https://github.com/user-attachments/assets/fff26b5b-c5c7-4eaf-952e-efdb9f43b6ba)

![WhatsApp Image 2024-05-18 at 13 14 44_8a46ed0f](https://github.com/user-attachments/assets/1709d8bd-ce44-4dac-963c-1033a315be88)

