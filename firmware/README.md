To test use:

```C
sudo mosquitto_sub -h localhost -p 1883 -u melvingust -P chihuahuaselvagem -t ctrl/state -t plant/state
sudo mosquitto_pub -h localhost -p 1883 -u melvingust -P chihuahuaselvagem -t ctrl/ref -m "a"
```


http://localhost:1880/dashboard/control
http://127.0.0.1:1880/#flow/35ad271e