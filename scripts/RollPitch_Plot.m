load data.txt
archivo=data;

t = archivo(:,1);
roll_angle = archivo(:,2);
pitch_angle = archivo(:,3);
yaw_heading = archivo(:,4);
roll_error = archivo(:,5);
pitch_error = archivo(:,6);
yaw_error = archivo(:,7);
roll_pid = archivo(:,8);
pitch_pid = archivo(:,9);
yaw_pid = archivo(:,10);
ref_roll = archivo(:,11);
ref_pitch = archivo(:,12);
ref_heading = archivo(:,13);

cutoff_freq = 10; % 10 Hz cutoff frequency
dt = 0.005;
rc = 1 / (2 * pi * cutoff_freq);
alpha = dt / (rc + dt);

filtered_yaw = zeros(size(t));
for i = 2:size(t,1)
    filtered_yaw(i) = alpha * yaw_heading(i) + (1 - alpha) * filtered_yaw(i-1);
end

KP = 1.5; KP_y = 2.0;
roll_pid_p = roll_error .* KP;
pitch_pid_p = pitch_error .* KP;
yaw_pid_p = yaw_error * KP_y;
% %%
% figure(1)
% plot(t, ref_roll,'k', t,roll_angle,'b', t,((-roll_pid)*0.5),'g', t,roll_error,'c'); ylim([-40,40])
% xlabel('Time(s)'); ylabel('Angles(deg)'); legend('Ref','roll_{ang}','roll_{pid}','roll_{er}'); grid;
% 
% figure(2)
% plot(t, ref_pitch,'k', t,pitch_angle,'b', t,pitch_pid,'g', t,pitch_error,'c'); ylim([-40,40])
% xlabel('Time(s)'); ylabel('Angles(deg)'); legend('Ref','pitch_{ang}','pitch_{pid}','pitch_{er}'); %grid;
% 
% figure(3)
% plot(t, ref_heading,'k', t,yaw_heading,'b', t,yaw_pid,'g', t,yaw_error,'c'); ylim([-400,400])
% xlabel('Time(s)'); ylabel('Heading(deg)'); legend('Ref','yaw_{ang}','yaw_{pid}','yaw_{er}'); %grid;
%%
figure(1)
subplot(12,1,1:2)
plot(t, ref_roll,'k', t,roll_angle,'b', t,-roll_error,'c'); ylim([ref_roll(1)-10,ref_roll(1)+10]);grid;
legend('Ref','roll_{ang}','roll_{er}','Location','northwest');
subplot(12,1,3:4)
plot(t,(-roll_pid),'g', t, -roll_pid_p,'b'); ylim([-50,50]);grid;
legend('roll_{pid}','P term','Location','northwest')
subplot(12,1,5:6)
plot(t, ref_pitch,'k', t,pitch_angle,'b', t,-pitch_error,'c'); ylim([ref_pitch(1)-10,ref_pitch(1)+10]);grid;
legend('Ref','pitch_{ang}','pitch_{er}','Location','northwest');
subplot(12,1,7:8)
plot(t,(-pitch_pid),'g', t, -pitch_pid_p,'b'); ylim([-50,50]);grid;
legend('pitch_{pid}','P term','Location','northwest')
subplot(12,1,9:10)
plot(t, ref_heading,'k', t,yaw_heading,'b', t,-yaw_error,'c'); ylim([ref_heading(1)-18.25,ref_heading(1)+18.25]);grid;
legend('Ref','yaw_{ang}','yaw_{er}','Location','northwest');
subplot(12,1,11:12)
plot(t,(-yaw_pid),'g', t, -yaw_pid_p, 'b'); ylim([-50,50]);grid;
legend('yaw_{pid}','P term','Location','northwest');