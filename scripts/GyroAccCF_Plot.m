load cfdata.txt
archivo=cfdata;

t = archivo(:,1);
acc_x = archivo(:,2);
gyro_x = archivo(:,3);
CF_x = archivo(:,4);
acc_y = archivo(:,5);
gyro_y = archivo(:,6);
CF_y = archivo(:,7);

%% X-Axis Calcs
CF_x_90 = zeros(size(t)); CF_x_90(1) = acc_x(1);
CF_x_92 = zeros(size(t)); CF_x_92(1) = acc_x(1);
CF_x_94 = zeros(size(t)); CF_x_94(1) = acc_x(1);
CF_x_98 = zeros(size(t)); CF_x_98(1) = acc_x(1);
CF_x_99 = zeros(size(t)); CF_x_99(1) = acc_x(1);

for i = 2:size(t,1)
    CF_x_90(i) = 0.90 * (CF_x_90(i-1) + (gyro_x(i)-gyro_x(i-1))) + (1-0.90) * acc_x(i);
    CF_x_92(i) = 0.92 * (CF_x_92(i-1) + (gyro_x(i)-gyro_x(i-1))) + (1-0.92) * acc_x(i);
    CF_x_94(i) = 0.94 * (CF_x_94(i-1) + (gyro_x(i)-gyro_x(i-1))) + (1-0.94) * acc_x(i);
    CF_x_98(i) = 0.98 * (CF_x_98(i-1) + (gyro_x(i)-gyro_x(i-1))) + (1-0.98) * acc_x(i);
    CF_x_99(i) = 0.99 * (CF_x_99(i-1) + (gyro_x(i)-gyro_x(i-1))) + (1-0.99) * acc_x(i);
end
%% Y-Axis Calcs
CF_y_90 = zeros(size(t)); CF_y_90(1) = acc_y(1);
CF_y_92 = zeros(size(t)); CF_y_92(1) = acc_y(1);
CF_y_94 = zeros(size(t)); CF_y_94(1) = acc_y(1);
CF_y_98 = zeros(size(t)); CF_y_98(1) = acc_y(1);
CF_y_99 = zeros(size(t)); CF_y_99(1) = acc_y(1);

for i = 2:size(t,1)
    CF_y_90(i) = 0.90 * (CF_y_90(i-1) + (gyro_y(i)-gyro_y(i-1))) + (1-0.90) * acc_y(i);
    CF_y_92(i) = 0.92 * (CF_y_92(i-1) + (gyro_y(i)-gyro_y(i-1))) + (1-0.92) * acc_y(i);
    CF_y_94(i) = 0.94 * (CF_y_94(i-1) + (gyro_y(i)-gyro_y(i-1))) + (1-0.94) * acc_y(i);
    CF_y_98(i) = 0.98 * (CF_y_98(i-1) + (gyro_y(i)-gyro_y(i-1))) + (1-0.98) * acc_y(i);
    CF_y_99(i) = 0.99 * (CF_y_99(i-1) + (gyro_y(i)-gyro_y(i-1))) + (1-0.99) * acc_y(i);
end
%% X-Axis Plot
figure(1)
plot(t, gyro_x,'b', t,acc_x,'r', t,CF_x,'k', t, CF_x_90,'g', t, CF_x_92,'cyan', t,CF_x_94,'yellow', t,CF_x_98,'magenta');
xlabel('Time(s)'); ylabel('Angles(deg)'); legend('Gyro','Acc','CFilter.96','CFilter.90','CFilter.92','CFilter.94','CFilter.98'); grid;

%% Y-Axis Plot
figure(2)
plot(t, gyro_y,'b', t,acc_y,'r', t,CF_y,'k', t, CF_y_90,'g', t, CF_y_92,'cyan', t,CF_y_94,'yellow', t,CF_y_98,'magenta');
xlabel('Time(s)'); ylabel('Angles(deg)'); legend('Gyro','Acc','CFilter.96','CFilter.90','CFilter.92','CFilter.94','CFilter.98'); grid;

%%
% figure(1)
% subplot(8,1,1:2)
% plot(t, ref_roll,'k', t,roll_angle,'b', t,-roll_error,'c'); ylim([-10,10])
% legend('Ref','roll_{ang}','roll_{er}');
% subplot(8,1,3)
% plot(t,(roll_pid),'g', t, roll_pid_p,'b'); ylim([-100,100])
% legend('roll_{pid}','proportional term')
% subplot(8,1,4:5)
% plot(t, ref_pitch,'k', t,pitch_angle,'b', t,-pitch_error,'c'); ylim([-10,10])
% legend('Ref','pitch_{ang}','pitch_{er}');
% subplot(8,1,6)
% plot(t,(pitch_pid),'g'); ylim([-100,100])
% legend('pitch_{pid}')
% subplot(8,1,7:8)
% plot(t, ref_heading,'k', t,yaw_heading,'b', t,-yaw_error,'c', t,(-yaw_pid),'g'); ylim([30,120])
% legend('Ref','yaw_{ang}','yaw_{er}','yaw_{pid}');