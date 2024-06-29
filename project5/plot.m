T = readtable('simulation.csv');
clf
hold on
plot(T.time, T.actual_roll);
plot(T.time, T.estimated_roll);
plot(T.time, T.reference_roll);
legend("r_{true}","r_{est}","r_{ref}");