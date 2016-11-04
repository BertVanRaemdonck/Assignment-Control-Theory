close all
clear all
clc
clear

rec_random1 = readlog('log_gpio_ramp2.xml');

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



%% Unwrapping the values of the encoder

tol_encoder = 32761;
random1_enc1_a = random1_enc1;
for i = length(random1_enc1)-1:-1:1
    
    Y = sign(random1_enc1);
    if Y(i)==1 && Y(i+1)==-1
        if random1_enc1(i) > 32700
            random1_enc1_a(i+1:end) = random1_enc1_a(i+1:end) + (2*tol_encoder);
        end
    end
    if Y(i)==-1 && Y(i+1)==1
        if random1_enc1(i) < -32700
            random1_enc1_a(i+1:end) = random1_enc1_a(i+1:end) - (2*tol_encoder);
        end
    end
end

figure()
subplot(3,1,1)
plot(random1_t, random1_enc1_a);
subplot(3,1,2)
plot(random1_t, random1_enc1);
subplot(3,1,3)
plot(random1_t, random1_enc1_a-random1_enc1);


random1_enc2_a = random1_enc2;
for i = length(random1_enc2)-1:-1:1
    
    Y = sign(random1_enc2);
    if Y(i)==1 && Y(i+1)==-1
        if random1_enc2(i) > 32700
            random1_enc2_a(i+1:end) = random1_enc2_a(i+1:end) + (2*tol_encoder);
        end
    end
    if Y(i)==-1 && Y(i+1)==1
        if random1_enc2(i) < -32700
            random1_enc2_a(i+1:end) = random1_enc2_a(i+1:end) - (2*tol_encoder);
        end
    end
end

figure()
subplot(3,1,1)
plot(random1_t, random1_enc2_a);
subplot(3,1,2)
plot(random1_t, random1_enc2);
subplot(3,1,3)
plot(random1_t, random1_enc2_a-random1_enc2);
        