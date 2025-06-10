# robotnik_email

The robotnik_email package, email server to send messages through ROS

## Installation

This package depens on the following packages:

- robotnik_msgs[🔗](https://github.com/RobotnikAutomation/robotnik_msgs)

```
git clone https://github.com/RobotnikAutomation/robotnik_msgs
```

- rcomponent[🔗](https://github.com/RobotnikAutomation/rcomponent)

```
git clone https://github.com/RobotnikAutomation/rcomponent
```

Install the package:

```
git clone https://github.com/RobotnikAutomation/robotnik_email.git
```

Install other ros dependencies:

```
rosdep install --from-path src --ignore-src -y -r
```

Build the package
```
catkin build
source devel/setup.bash
```


## SMTP Configuration

In order to use this package a SMTP server is required. There are two modes of use:

- Working with a SMTP provided by a third party. For example, a customer or a company.

- Working with your own SMTP, for testing purposes only.

### a) SMTP Third Party

The customer has to give us acces to work with his SMTP. You have ask for ```user```, ```password```, ```from email```, ```SMTP server``` and ```port```. In this example credentials are not needed:

```
User: ---
Password: --- 
From email:  name@company.com
SMTP server: 10.100.100.10
Port: 25
```

### b) SMTP on your own

This is the recommended way to familiarize yourself with this package.

Use a google account without double verification and watch this video to configure your own SMTP: https://www.youtube.com/watch?v=ql5Dex4m40w

At the end of the video you will be able to fill all the fields. This is an example:

```
User: username@gmail.com
Password: yourpassword 
From email:  username@gmail.com
SMTP server: smtp.gmail.com
Port: 587
```

Additionally, you can test if your SMTP server is working properly by sending an email through the following link. Select ```Tls``` as securty field: https://www.gmass.co/smtp-test


## Package configuration

This package needs the SMTP server information to send emails. So that, edit the following file:

```
config/smtp_config.yaml
```

Fill the fields with your SMTP configuration like in this example:

```
smtp:
  server: 'smtp.gmail.com'
  port: 587
  use_authentication: true
  username: 'username@gmail.com'
  password: 'yourpassword'
  sender: 'username@gmail.com'
  default_recipients:
    - 'recipient1@domain.com'
    - 'recipient2@domain.com'
```

The following table describes each parameter:

| Name  | Type  | Default  | Description | 
|---|---|---|---|
| server  | string  | smtp.gmail.com  | Server address  |
| port  | int  | 587  | Server port  |
| use_authentication  | bool  | False  | Some SMTP servers needs credentials  |
| username  | string  | username  | Name credentials   |
| password  | string  | password  | Password credentials  |
| sender  | string  | sender@domain.com    | Email from where the message is sent  |
| default_recipients  | string[]  | recipient@domain.com  |  If the recipients field is empty, gets these recipients  |
| time_between_emails | int | 0 | Time between one email and another to avoid overloads (not implemented yet) |


## Bringup

Launch the package:

```
roslaunch robotnik_email email.launch
```

Send a email to the default recipieints:
```
rosservice call /robotnik_email/send_email "recipients: []
status: {id: 0, description: '', type: '', message: ''}
uuid: ''
datetime: ''
files_to_upload: []"
```

Send a email to specific recipieints:
```
rosservice call /robotnik_email/send_email "recipients: ['recipient1@gmail.com', 'recipient2@gmail.com0']
status: {id: 0, description: '', type: '', message: ''}
uuid: ''
datetime: ''
files_to_upload: []"
```

Send a email to specific recipieints including some file attachments:
```
rosservice call /robotnik_email/send_email "recipients: ['recipient1@gmail.com', 'recipient2@gmail.com0']
status: {id: 0, description: '', type: '', message: ''}
uuid: ''
datetime: ''
files_to_upload: ['image1.png', 'video1.mkv']"
```




## 1. smtp_manager

It sends emails using a SMTP server

### 1.1 Parameters

 * **server** (string, default: smtp.gmail.com):

   SMTP server address

 * **port** (int, default: 587):
   
   SMTP server port  

 * **sender** (string, default: ):
   
   Email from where the message is sent

 * **use_authentication** (bool, default: False):
   
   Enable credentials when a SMTP needs them 

 * **ssl** (bool, default: False):
   
   Enable SSL connection

 * **tls** (bool, default: False):
   
   Enable TLS connection

 * **timeout** (int, default: 10):
   
   Timeout to connect to the SMTP server

 * **tls** (bool, default: False):
   
   Enable TLS connection

 * **username** (string, default: username):
   
   Name credentials

 * **password** (string, default: password):
   
   Password credentials

 * **default_recipients** (string[], default: []):
   
   Default recipiens if the recipients field is empty when a message is sent.

 * **time_between_emails** (float, default: 5):
   
   Time (in seconds) between one email and another to avoid overloads

 * **include_detailed_info** (bool, default: True.):
    
    Include detailed information in the email. The detailed information is taken from the rosservice send_email

 * **auto_generate_uuid_datetime** (bool, default: True):
    
    Automatically generate the uuid and datetime fields in the email in case it is not provided by the user
 
 * **queue_max_len** (int, default: 1000):
    
    Max length of queue where the messages are stored until they are sent
        

### 1.2 Published Topics

* ~/state (robotnik_msgs/State):
  Node health

* ~/queue_size (std_msgs/UInt32)
  Size of the queued messages to be sent

### 1.3 Services
* ~/send_email (robotnik_alarms_msgs/SendAlarms)
The service queues the email to be sent to the recipients with the status message and the files to upload.  
Example:
  ```
    rosservice call /smtp_manager/send_email "recipients: ['info@robotnik.es']
    status: {id: 0, description: 'Temperature Alarm', type: '', message: 'The alarm temperature has been reached'}
    uuid: ''
    datetime: ''
    files_to_upload: ['/home/robot/Pictures/camera-1.png', '/home/robot/Pictures/camera-2.png']" 
  ```
  