load data.txt
archivo=data;

mag_x  = archivo(:,1);
mag_y = archivo(:,2);

plot(mag_x,mag_y,'g*')
xlabel('MAG_Y(uT)'); ylabel('MAG_Z(uT)'); grid;