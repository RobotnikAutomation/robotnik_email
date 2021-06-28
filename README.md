# robotnik_email

The robotnik_email package, email server to send messages through ROS

## Installation

Clone this package on your workspace

```
cd ~/catkin_ws/src
git clone https://github.com/RobotnikAutomation/robotnik_email.git
cd  ~/catkin_ws
catkin_build
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
| default_recipients  | string[]  | recipient@domain.com  | Default recipiens if the recipients field is empty when a message is sent  |
| time_between_emails | int | 0 | Time between one email and another to avoid overloads |




## Bringup

Launch the package:

```
roslaunch robotnik_email email.launch
```

Send a email to the default recipieints:
```
rosservice call /smtp_manager/send_email "recipients:
- ''
subject: 'Robotnik Email'
message: 'Hello world'" 
```

Send a email to specific recipieints:
```
rosservice call /smtp_manager/send_email "recipients:
- 'someuser@domain.com'
subject: 'Robotnik Email'
message: 'Hello world'" 
```




## 1. smtp_manager

It sends emails using a SMTP server

### 1.1 Parameters

* ~smtp/server (string, default: smtp.gmail.com):
   SMTP server address

 * ~smtp/port (int, default: 587):
   SMTP server port  

 * ~smtp/sender (string, default: sender@domain.com):
   Email from where the message is sent

 * ~smtp/use_authentication (bool, default: False):
   Enable credentials when a SMTP needs them 

 * ~smtp/username (string, default: username):
   Name credentials

  * ~smtp/password (string, default: password):
   Password credentials

  * ~smtp/default_recipients (string[], default: recipient@domain.com):
    Default recipiens if the recipients field is empty when a message is sent.

  * ~smtp/time_between_emails (int, default: 0):
    Time between one email and another to avoid overloads
 

### 1.2 Published Topics

* smtp_manager/state (robotnik_msgs/State):
  Node health

### 1.3 Services
* smtp_manager/send_email (robotnik_email/SendEmail)
  Service to send emails
