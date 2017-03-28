%%

%Rz = [0 -1 0 ; 1 0 0 ; 0 0 1 ]; 

%Rx = [1 0 0 ; 0 0 -1 ; 0 1 0];

%Rz*Rx

Ry = [0 0 1; 0 1 0; -1 0 0]
%%

ax_data = mean(ax.Data);
ay_data = mean(ay.Data);
az_data = mean(az.Data);


%%
a = [ax_data ; ay_data ; az_data];
a_norm = norm(a)


%%

r = vrrotvec(a,[0; 0; a_norm])
m = vrrotvec2mat(r)

a_new = m * a;

%% Plot
plot(ax.Data)
grid on; hold on
plot(ay.Data)
plot(az.Data)

legend('ax','ay','az')
%%
a_serie = [ax.Data' ; ay.Data' ; az.Data'] ;


test = m* a_serie;

plot(test(1,:))
grid on; hold on
plot(test(2,:))
plot(test(3,:))
legend('ax','ay','az')