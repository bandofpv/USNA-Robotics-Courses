classdef gimbalClass < matlab.mixin.SetGet
    properties
        S,accel,gyro,mag,eul;
        cleanupObj;
        fig,axs;
        accel_txt,gyro_txt,eul_txt,mag_txt;
        ticker,time;
        t_imu,t_mag,t_eul;
        imu_seq,mag_seq,eul_seq;
        acc_data,gyro_data,eul_data;
        imu_dt;
        imu_data_flag;
        eul_data_flag;

    end
    methods

        function obj = gimbalClass(port)
            obj.time = tic;
            obj.imu_dt =tic;
            obj.t_imu = [];
            obj.t_mag = [];
            obj.t_eul = [];
            obj.accel = [0,0,0];
            obj.eul = [0,0,0];
            obj.imu_data_flag = false;
            obj.eul_data_flag = false;

            % Initialize Connection to gimbal
            disp('INITIALIZING CONNECTION TO BOARD')

            COMMPORTNUM = strcat( 'COM',num2str(port));
            % Cleaning up old ports
            a = instrfind('Port' , COMMPORTNUM);
            if ~isempty(a)
                disp('DELETING OLD SERIAL PORT INFO. ')
                fclose(a);
                pause(0.1);
                delete(a);
                pause(0.1);
            end

            % set up new port
            disp('SETTING UP NEW PORT.')
            obj.S = serialport(COMMPORTNUM, 921600);

            obj.S.configureTerminator("CR/LF");
            obj.S.configureCallback("terminator",@obj.serialParse);

            pause(0.1)
            disp('OPENING PORT.')
            %             fopen(obj.S);
            pause(1)
            obj = initWindow(obj);
            obj.ticker = timer('Period',0.02,'ExecutionMode','fixedRate');
            obj.ticker.TimerFcn = @obj.updateDisplay;
            start(obj.ticker)
            

        end
        function obj = initWindow(obj)
            obj.fig = figure(10);
            set(obj.fig,'Name','Gimbal Status Window');
            obj.axs = axes('Parent',obj.fig,'Visible','Off');
            str_acc = '[$\dot{u}$ $\dot{v}$ $\dot{w}$]';
            str_gyro = '[p q r]';
            str_eul = '[$\phi$ $\theta$ $\psi$]';

            text(obj.axs,0.25,0.92,'Accelerometer','Fontsize',24,'Interpreter','latex')
            text(obj.axs,0.05,0.8,str_acc,'FontSize',24,'Interpreter','latex')

            text(obj.axs,0.3,0.62,'Gyroscope','Fontsize',24,'Interpreter','latex')
            text(obj.axs,0.05,0.5,str_gyro,'FontSize',24,'Interpreter','latex')
            
            text(obj.axs,0.3,0.32,'Euler Angles','Fontsize',24,'Interpreter','latex')
            text(obj.axs,0.05,0.2,str_eul,'FontSize',24,'Interpreter','latex')


            obj.accel_txt = text(0.3,0.8,'','FontSize',24,'Interpreter','latex','Parent',obj.axs);
            obj.gyro_txt = text(0.3,0.5,'','FontSize',24,'Interpreter','latex','Parent',obj.axs);
            obj.eul_txt = text(0.3,0.2,'','FontSize',24,'Interpreter','latex','Parent',obj.axs);
        end
        function obj = serialParse(obj,S,event)

            %             if(S.BytesAvailable > 10)
            msg = readline(S); %string from board

            if startsWith(msg, '$IMU')
                data = sscanf(msg, '$IMU,%f,%f,%f,%f,%f,%f,%f');
                if(numel(data)==7)
                    % dt = 1/(data(1)-obj.t_imu);
                    obj.t_imu = data(1);
                    obj.accel = data(2:4);
                    obj.gyro = data(5:7);
                    obj.imu_data_flag = true;
                end
            end
            if startsWith(msg, '$EUL')

                data = sscanf(msg, '$EUL,%f,%f,%f,%f');
                if(numel(data)==4)
                    obj.t_eul = data(1);
                    obj.eul = data(2:4);
                    obj.eul_data_flag = true;
                end
            end            

        end
        function [p,q,r]=getGyroData(obj)
            p = obj.gyro(1);
            q = obj.gyro(2);
            r = obj.gyro(3);
        end
        function [t,ud,vd,wd,p,q,r]=getImuData(obj)
            t = obj.t_imu;
            ud = obj.accel(1);
            vd = obj.accel(2);
            wd = obj.accel(3);
            p = obj.gyro(1);
            q = obj.gyro(2);
            r = obj.gyro(3);
        end
        function [mx,my,mz]=getMagData(obj)
            mx = obj.mag(1);
            my = obj.mag(2);
            mz = obj.mag(3);
        end
        function [t,roll,pitch,yaw] =getEulerData(obj)
            t = obj.t_eul;
            roll = obj.eul(1);
            pitch = obj.eul(2);
            yaw = obj.eul(3);
        end
        function updateDisplay(obj,ticker,event)
            %             disp('window update')
            %                imu_hz = 1/mean(diff(obj.t_imu))
            if ishandle(obj.fig)
                if(~isempty(obj.accel))
                    acc_str = sprintf('= %.3f %.3f %.3f',obj.accel(1),obj.accel(2),obj.accel(3));
                    set(obj.accel_txt,'String',acc_str);
                end
                if(~isempty(obj.eul))
                    eul_str = sprintf('= %.3f %.3f %.3f',obj.eul(1),obj.eul(2),obj.eul(3));
                    set(obj.eul_txt,'String',eul_str);
                end
                if(~isempty(obj.gyro))
                    gyro_str = sprintf('= %.3f %.3f %.3f',obj.gyro(1),obj.gyro(2),obj.gyro(3));
                    set(obj.gyro_txt,'String',gyro_str);
                end
                %if(~isempty(obj.mag))
                %    mag_str = sprintf('= %.0f %.0f %.0f',obj.mag(1),obj.mag(2),obj.mag(3));
                %    set(obj.mag_txt,'Position',[0.4,0.5],'String',mag_str);
                %end
                %drawnow
            else
                obj.delete()
            end
        end
        function delete(obj)
            disp('Closing Port.')
            stop(obj.ticker)
            delete(obj.ticker)
            pause(1)
            if isgraphics(obj.fig)
                close(obj.fig)
            end
            delete(obj.S)
            clear obj.S
        end

        function clean(obj)
            disp('Clean Function Called')
        end

    end
end