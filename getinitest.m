function [delta,phi] = getinitest(Lpts, Nc)
    num = length(Nc);
    A = zeros(num,9);
    B = zeros(num,1);
    % Pf = [X,Z,1]';
%     Lpts(2,:) = Lpts(3,:);
%     Lpts(3,:) = 1;

    for i=1:num
        Na1 = (Nc(1,i).*Lpts(1,i));
        Na2 = (Nc(1,i).*Lpts(2,i));
        Na3 = (Nc(1,i));
        Nb1 = (Nc(2,i).*Lpts(1,i));
        Nb2 = (Nc(2,i).*Lpts(2,i));
        Nb3 = (Nc(2,i));
        Nc1 = (Nc(3,i).*Lpts(1,i));
        Nc2 = (Nc(3,i).*Lpts(2,i));
        Nc3 = (Nc(3,i));

        A(i,:)=[Na1 Na2 Na3 Nb1 Nb2 Nb3 Nc1 Nc2 Nc3]; % num by 9
        B(i,1) = norm(Nc(:,i))^2; % num by 1
    end
    
    h = A\B;
    H = [h(1:3,1),h(4:6,1),h(7:9,1)]'; % 3 by 3 matrix

    phi = [H(:,1),cross(-H(:,1),H(:,2)),H(:,2)]'; % orientation
    delta = -[H(:,1),cross(-H(:,1),H(:,2)),H(:,2)]'*H(:,3); % position

end