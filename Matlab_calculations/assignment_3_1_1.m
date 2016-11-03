close all
clear

rec_random1 = readlog('log_gpio_random1.xml');

random1_t = rec_random1.getData('time');
random1_v = rec_random1.getData('Voltage');
random1_enc1 = rec_random1.getData('Encoder1');
random1_enc2 = rec_random1.getData('Encoder2');

figure()
subplot(3,1,1)
plot(random1_t, random1_v);
subplot(3,1,2)
plot(random1_t, random1_enc1);
subplot(3,1,3)
plot(random1_t, random1_enc2);