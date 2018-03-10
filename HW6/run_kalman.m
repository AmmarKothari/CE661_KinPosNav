function filtered_h_el = run_kalman(vel_samples, Q, R, fig_fn)
    tIndx = 2:2:length(vel_samples)*2;   % Sample interval is 2 sec
    Phi = [1, 2; 0, 1];
    H = [0, 1];
    
    KF = kalman_traj(Phi, Q, H, R);
    KF = KF.setInitialValues([0;0], [0,0;0,0]);
    for i =1:length(vel_samples)
        xSample = vel_samples(i);

        % Note: in the line of code below, you should replace the call to
        % "kalman_traj_hw5) with a call to the Kalman filter function you write
        % for this HW
    %     [x_kalman, cov, Kalman_gain] = kalman_traj(xSample);
        [KF, x_kalman, cov, Kalman_gain] = KF.update(xSample);

        filtered_h_el(i) = x_kalman(1);
        filtered_vel(i) = x_kalman(2);
        Kg(i,:) = Kalman_gain;
        P_norm(i) = norm(cov);
    end

    % Create plots of the output trajectory
    hold on
    subplot(2,1,1)
    plot(tIndx,filtered_h_el,'r','LineWidth',2.5)
    title('Kalman Filter Results')
    xlabel('Time (sec)');
    ylabel('Altitude (m)');
    legend('Kalman filtered trajectory')
    subplot(2,1,2)
    plot(tIndx,filtered_vel,'g','LineWidth',2.5)
    xlabel('Time (sec)');
    ylabel('Velocity (m/s)');
    hold on
    plot(tIndx,vel_samples)
    legend('Kalman filtered','Noisy Velocity Samples')
    saveas(gcf,fig_fn)
    close all;
end