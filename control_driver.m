%%%%%%%%%%%%%%%%%%%
% ASF HPZ 8 modules, sun tracking, 17.03.2016
% Update 10.04.2016
% Update 02.08.2016., changing to calculations with absolute angles
% Update 31.08.2016: Expanding for 16 modules
%%%%%%%%%%%%%%%%%%%

% decomposing the state vector
check_pr_in_ch = state(1,1:3);
waiting                 = state(1,4);
waiting_time        = state(1,5);
pON                      = state(1,6);
pOFF                     = state(1,7);
horiz_regulation =  state(1,8);
vertical_regulation = state(1,9);
finshed_vert_reg = state(1,10);
finshed_horiz_reg = state(1,11);
regulation_ongoing = state(1,12) ;
watchdog_Reg = state(1,13:18); 
specialR = state(1,19:21);
delta_deg_in_V_reached = state(1,22);
delta_deg_in_H_reached = state(1,23);
inter_ref_valid = state(1,24);
refV = state(1,25);
refH = state(1,26);
refV_angle_old = state(1,27);
refH_angle_old = state(1,28);
exit_Saturation = state(1,29);
indAct = state(1,30);
flag_MActFinished = state(1,31);
first_run_COM_pr_checked = state(1,32); 
nextAct = state(1,33); 
refChanged = state(1,34); 
SunTrSeq_old = state(1, 35); 
first_wait_run = state(1, 36);
checking_pr_in_ch = state(1, 37);
out_of_limits = state(1, 38); L = 39;
Saturated = state(1, L:1: L+N_Modules*3-1); %L = L+N_Modules*3;
% Oriented = state(1, L:1: L+N_Modules*3-1);
% flag indicating that the common pressure in the chamber has been checked and if necessary reduced
k_Ch = L+N_Modules*3; % ok
Chs = state(1,k_Ch:1:k_Ch+N_Modules*3-1);


% Decomposing the HoNR_Param vector
% HoNR_Param - t_ON, t_OFF, p_ON, p_OFF, N_steps
t_ON = HoNR_Param(1);
t_OFF = HoNR_Param(2);
p_ON = HoNR_Param(3)/100;
p_OFF = HoNR_Param(4)/100;
N_steps = HoNR_Param(5);
pr_delta = HoNR_Param(6)/100;
refV_GUI = HoNR_Param(7);
refH_GUI = HoNR_Param(8);
refV_GUI_Slider = HoNR_Param(9);
refH_GUI_Slider = HoNR_Param(10);
ActRange_MaxH = HoNR_Param(11);
ActRange_MinH = HoNR_Param(12);
ActRange_MaxV = HoNR_Param(13);
ActRange_MinV = HoNR_Param(14);
delta_angles = HoNR_Param(15);
MaxZigZagAngle = HoNR_Param(16);
H_offset = HoNR_Param(17);
BaseOrientV = HoNR_Param(18);
BaseOrientH = HoNR_Param(19);
SunTr_UpdTime = HoNR_Param(20);
SunTr_Method = HoNR_Param(21);
DELTA_ANGLE_ADDITIONAL_LIMIT = HoNR_Param(22);

% PARAMETERS of the algorithm
% constants
pr_low_limit = 0.3;
time_const_alternate_Chs = 0.4; % in sec
wait_const   = 1 / scan_t_in_s; % first is the waiting time in sec, the second is the loop period in sec
watchdogCONTS = 3;
MID_DELTA_DEG = MaxZigZagAngle;
WATCHDOG_CONST = 1 / scan_t_in_s;
MAX_SATUR_TRIALS = 2;
% DELTA_ANGLE_ADDITIONAL_LIMIT = 5; % in degrees, how much can dirft from maximum angles 
% defined in gui


% Initialisation of variables
region = 0; % control region
stop_regulation = 0;
tr_seq=0;
temp1=0;
ledX = 0;
bla =0;
led_alg1 = 0;
Ch1 = 0;
Ch2 = 0;
Ch3 = 0;
refValg = 0;
H_or_V = 0; % flag to allow for only one horizontal or vertical regulatin per pass and not vertical 
% and in the same pass immediately horizontal

% Calculating common V offset
CommonV_offset = sum(V_offset_vect)/sum(V_offset_vect~=0);

%%%%%%%%%%%%%%%%%%%%%%%% MAIN CODE %%%%%%%%%%%%%%%%%%%%%%

%%% Differentiation between different Orientation Control Strategies
%%
%% Control Method 0: All modules the same angle
%% Control Method 1: Sun tracking 1 day, going to ref. pos.

% sequential control of the actuators
if indAct == 0 % indAct counter -- index actuator starts from 1
   indAct = 1;
end

if (SunTr_Method == 0) %% Control Method 0: All modules the same angle
     % Get facade angles (V&H, the same angle for the whole facade) from GUI
      refV_FINAL = refV_GUI;
      refH_FINAL = refH_GUI;
elseif (SunTr_Method == 1) %% Control Method 1: Sun tracking 1 day, going to ref. pos.
      if (t_in_s <= SunTr_MatrixDim(1)*SunTr_UpdTime*60)  % if solar tracking sequence not finished
         % SunTr_UpdTime is in minutes
         % Read sun angles(V&H, the same angle for the whole facade) from SunTrMatrix
          SunTr_Seq = floor(t_in_s/(SunTr_UpdTime*60)) + 1; % index of tracking sequence OK
          SunTr_Mode = mod(t_in_s,SunTr_UpdTime*60); 
          % 1st half of the sun tracking interval -> do sun tracking
          % 2nd half of the sun tracking interval -> go to the base position
           if (SunTr_Mode <= SunTr_UpdTime*60) % add "/3"; 1st half of the sun tr period --> do sun tracking
                 refV_FINAL = SunTr_Matrix(1+(SunTr_Seq-1)*2); % - 90 + CommonV_offset;
                 % refV_FINAL = (SunTr_Matrix(1+(SunTr_Seq-1)*2)  - 90 + V_offset_vect(indAct) );
                 refH_FINAL = -SunTr_Matrix(SunTr_Seq*2)
                 % Respecting actuator range
                  if (refV_FINAL > CommonV_offset + ActRange_MaxV)
                      refV_FINAL = CommonV_offset + ActRange_MaxV;
                  end
                  if (refV_FINAL < CommonV_offset + ActRange_MinV)
                      refV_FINAL = CommonV_offset + ActRange_MinV;
                      if (refV_FINAL < 0)
                             refV_FINAL = 0;
                       end
                  end
                  if (refH_FINAL > ActRange_MaxH)
                     refH_FINAL = ActRange_MaxH;
                  end
                  if (refH_FINAL < ActRange_MinH)
                    refH_FINAL = ActRange_MinH;
                 end
            %elseif( SunTr_Mode <= SunTr_UpdTime*60*2/3 ) % 2nd part of the sun tr. interval, go to the base position
             %     refV_FINAL = BaseOrientV + CommonV_offset ;
             %     refH_FINAL = BaseOrientH;
            %else % 3rd part of the sun tracking interval, going to yearly optimal angle
             %     angleV_yearly_optimal = 45;
              %    if (CommonV_offset + ActRange_MaxV > angleV_yearly_optimal)
            %             refV_FINAL = angleV_yearly_optimal;
            %      else
             %            refV_FINAL = CommonV_offset + ActRange_MaxV;
             %     end
              %    refH_FINAL = BaseOrientH;
           end % of  if (SunTr_Mode...)
           % SOME ADDITIONAL SAFETY MECHANISMS, 06.09.2016
            % limiting vertical angle
           refH_MAX = ActRange_MaxH + DELTA_ANGLE_ADDITIONAL_LIMIT;
           refH_MIN = ActRange_MinH - DELTA_ANGLE_ADDITIONAL_LIMIT;           
           refV_MAX = CommonV_offset + ActRange_MaxV + DELTA_ANGLE_ADDITIONAL_LIMIT;
           refV_MIN = CommonV_offset + ActRange_MinV - DELTA_ANGLE_ADDITIONAL_LIMIT;
           if(refV_FINAL < 10)
                 refV_FINAL = 10;
                 refV_MIN = 10 -  DELTA_ANGLE_ADDITIONAL_LIMIT;
           end
            
       else %  of  if (t_in_s > SunTr_MatrixDim(1)*SunTr_UpdTime*60)
            stop_regulation = 1; % stop sun tracking; sun tracking finished
       end %  of  if (t_in_s > SunTr_MatrixDim(1)*SunTr_UpdTime*60)
       % RESETING THE INDEX NUMBER OF ACTUATOR after the ref angle value has changed
if  (refV_angle_old ~= refV_FINAL || refH_angle_old ~= refH_FINAL )
     indAct = 1;
      finshed_vert_reg = 0;
      finshed_horiz_reg = 0;
      vertical_regulation = 0;
      horiz_regulation = 0;
      regulation_ongoing = 0;
      first_run_COM_pr_checked = 0;
       inter_ref_valid = 0;
      waiting = 0;
      Saturated = zeros(1, N_Modules*3);
      % Oriented = zeros(1, N_Modules*3);
       nextAct = 0;
      out_of_limits = 0;
     check_pr_in_ch = zeros(1,3);  
end
 end % if (SunTr_Method )

if (stop_regulation == 0) % angle regulation/sun tracking ongoing

% checking if there is a change in reference
% if (SunTr_Method == 1) %% Control Method 0: All modules the same angle


%elseif (SunTr_Method == 1)
%if  (SunTrSeq_old ~= refV_FINAL || refH_angle_old ~= refH_FINAL )
%     indAct = 1;
%             finshed_vert_reg = 0;
%             finshed_horiz_reg = 0;
%             vertical_regulation = 0;
%             horiz_regulation = 0;
%             regulation_ongoing = 0;
%             first_run_COM_pr_checked = 0;
%             inter_ref_valid = 0;
 %     waiting = 0;
%end
%end

if (NextActPlus == 1)
      nextAct =nextAct + 1;
end
if nextAct % go to the next actuator
      nextAct = 0;
      indAct = indAct + 1;
      if ( indAct > N_Modules) % if the index of actuators is outside of the range, reset it to 1
            indAct = 1;
      end
      waiting = 0;
      first_run_COM_pr_checked = 0;
      check_pr_in_ch = zeros(1,3);
      watchdog_Reg = zeros(1,6); % three regions, vert and horiz control
      % checking that the module is selected for regulation on the UI
       Saturated = zeros(1,N_Modules*3);
       out_of_limits = 0;
end

% find the index of an active module
if (ModuleSel(indAct) == 0)
           indAct = indAct + 1;
           if ( indAct > N_Modules)
                indAct = 1;
            end
           while( ModuleSel(indAct) == 0 && indAct <N_Modules)
               indAct = indAct + 1;
           end
            if (indAct == N_Modules  && ModuleSel(indAct) == 0)
               indAct =  1;  % reset the acutator counter
           end
end

% reading out previously stored Ch's ON/OFF values
Ch1 = Chs(1, (indAct-1)*3+1);
Ch2 = Chs(1, (indAct-1)*3+2);
Ch3 = Chs(1, (indAct-1)*3+3);

% Calculating measured angles with offset for the current actuator
if (indAct <= 8)
     angleH = Pitches1(indAct);
     angleV = Rolls1(indAct);
else
     angleH = Pitches2(indAct-8);
     angleV = Rolls2(indAct-8);
end
% older: angleH = RPY((indAct-1)*3+2) - H_offset_vect(indAct);
%angleV = -( RPY((indAct-1)*3+1) - V_offset_vect(indAct) );
%%%%%%%%%%
% Update 18.04.2016, to get actuators vertically aligned
% older angleV = 90 - RPY((indAct-1)*3+1);


%% Changed to get all actuators vertically aligned****
% angleV = -( RPY((indAct-1)*3+1) - CommonV_offset );

% calculating delta angles
deltaV = angleV - refV_FINAL;
deltaH = angleH - refH_FINAL;
k_Zig_Zag = abs(deltaV/deltaH);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%% ALGORITHM 1: similar to ICRA 2016 %%%%%%%%%%

%%%  start regulation 
if (waiting)
    waiting_time = waiting_time + 1;
    if (waiting_time > wait_const)
         waiting = 0;
         waiting_time = 0;
         if(checking_pr_in_ch)
               first_wait_run = 1;
         end
     end % of if (waiting_time > wait_const)
else % NO WAITING, START CONTROL
           if (~first_run_COM_pr_checked) % if pressure in COMmon chamber hasn't been checked
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ledX = 200;
                 if(pr> maxPr( (indAct-1)*3 + 1) || pr> maxPr ( (indAct-1)*3 + 2)  || pr> maxPr ( (indAct-1)*3 + 3) ) 
                    % release pressure from the common chamber
                    pON = 0; pOFF = 1;  Ch1 = 0;  Ch2 = 0; Ch3 = 0;
                    waiting = 1;
                    %nextAct = 1; % if pr in common chamber is too big, release the pressure and go to the next act. 
                 else
                     first_run_COM_pr_checked = 1;
                 end

           else % of if (~first_run_com_pr_check) ; pressue in common chamber withing the limits,
           % READY TO START REGULATION
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
                  if(~regulation_ongoing)
% || refV_angle_old ~= refV_angle || refH_angle_old ~= refH_angle )
            ledX = 400;
                          % check if no regulation is ongling to intialise one
                          %led_alg1 = 1;
                          % check if the measured Vangle and Hangle are outside of the refernce band
                    %      if ( refV_angle_old ~= refV_angle || refH_angle_old ~= refH_angle )
                   %             inter_ref_valid = 0;
                   %             first_run_COM_pr_checked = 0;
                   %             regulation_ongoing = 0;
                  %        end

            % CHECKING IF REGULATION IS NEEDED
                   if (abs(deltaV) > 0.5*delta_angles) % start vertical regulation
                          regulation_ongoing = 1;
                          vertical_regulation = 1;
                          horiz_regulation = 0;
                          finshed_vert_reg = 0;
                          finshed_horiz_reg = 0;
                   elseif  (abs( deltaH) > 0.5*delta_angles) % start horizontal regulation
                          regulation_ongoing = 1;
                          vertical_regulation = 0;
                          horiz_regulation = 1;
                          finshed_vert_reg = 0;
                          finshed_horiz_reg = 0;
                  else
                          % set finished regulation flags in orther to go to the next actuator at the end of the code
                          finshed_vert_reg = 2; 
                          finshed_horiz_reg  = 2;
                   end %of (abs(deltaV) > 0.5*delta_angles) % start vertical regulation
                   inter_ref_valid = 0;

 %                 delta_deg_in_V_reached = 0;
  %                delta_deg_in_H_reached = 0;
   %               if (refV_angle_old ~= refV_angle || refH_angle_old ~= refH_angle )
   %                       if(pr> maxPr(indAct,1) || pr> maxPr(indAct,2) || pr> maxPr(indAct,3)) 
    %                           % release pressure from the common chamber
     %                          pON = 0; pOFF = 1;  Ch1 = 0;  Ch2 = 0; Ch3 = 0;
     %                          waiting = 1;
      %                         exit_Saturation = 1;
      %                     else
     %                         exit_Saturation = 0;
     %                      end
    %               end
     %              if nextAct
     %                   indAct = indAct + 1;
     %                   first_run_com_pr_check = 0;
     %                    inter_ref_valid = 0;
   %                     regulation_ongoing = 0;
   %                end
    %           end


          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          % IMPLEMENTING ZIG-ZAG REFERNCE ADJUSTMENT
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          % deltaV = angleV - refV;
          % deltaH = angleH - refH;
          if (~inter_ref_valid) % if one of the refernces is not checked, do it here
                % maximal allowed difference between the current angle and the refernce is MID_DELTA_DEG
                % Given deltaV and deltaH, we can calulate in which angle we will reach MID_DELTA_DEG,
                % while leaving the other angle at some smaller value. So let's check the ratio between
                % deltaV and deltaH
                refV = refV_FINAL;
                refH = refH_FINAL;
                k_Zig_Zag = abs(deltaV/deltaH);
                 if(k_Zig_Zag >= 1) 
                       % deltaV >= deltaH, so we should constrain deltaV to either final reference or MID_DELTA_DEG
                       if (abs(deltaV) > MID_DELTA_DEG )
                               refV = angleV - sign(deltaV)*MID_DELTA_DEG;
                              %delta_deg_in_V_reached = 0;
                              V_limit = MID_DELTA_DEG;
                       else % if the current angle difference is allowed, set it to the final reference value
                               refV = refV_FINAL; % final angle comanded
                               %delta_deg_in_V_reached = 1;
                               V_limit = deltaV;
                       end % end of if (deltaV > MID_DELTA_DEG)       
                       % constraining horizontal reference
                       if (abs(deltaH) > MID_DELTA_DEG )
                               refH = angleH - sign(deltaH)*abs(V_limit)/k_Zig_Zag;
                              %delta_deg_in_H_reached = 0;
                       else % if the current angle difference is allowed, set it to the final reference value
                               refH = refH_FINAL; % final angle comanded
                               %delta_deg_in_H_reached = 1;
                       end % end of if (deltaV > MID_DELTA_DEG)       
                else % of if (k_Zig_Zag >= 1) 
                       % deltaV < deltaH, so we should constrain deltaH to either final reference or MID_DELTA_DEG
                       if (abs(deltaH) > MID_DELTA_DEG )
                              refH = angleH - sign(deltaH)*MID_DELTA_DEG;
                              %delta_deg_in_H_reached = 0;
                              H_limit = MID_DELTA_DEG;
                       else % if the current angle difference is allowed, set it to the final reference value
                               refH = refH_FINAL; % final angle comanded
                               %delta_deg_in_H_reached = 1;
                               H_limit = deltaH;
                       end % end of if (deltaH > MID_DELTA_DEG)       
                       % constraining vertical reference
                       if (abs(deltaV) > MID_DELTA_DEG )
                               refV = angleV - sign(deltaV)*abs(H_limit)*k_Zig_Zag;
                              %delta_deg_in_V_reached = 0;
                       else % if the current angle difference is allowed, set it to the final reference value
                               refV = refV_FINAL; % final angle comanded
                               %delta_deg_in_V_reached = 1;
                       end % end of if (deltaV > MID_DELTA_DEG)       
                end % of if(k_Zig_Zag >= 1) 
           inter_ref_valid = 1;
           end % of   if (~inter_ref_valid); at the end of this line there is an inter ZigZag reference formed in 
           % any case         

          else %  of if(~regulation_ongoing), i.e. REGULATION IS ONGOING HERE
          
           refValg = refV -  V_offset_vect(indAct);

           % checking if this actuator is already saturated
          if (Saturated((indAct-1)*3 + 1) + Saturated((indAct-1)*3 + 2) +  Saturated((indAct-1)*3 + 3)  >=2 )
                  nextAct = 1;
          else % do control normaly 
           %%% Determining the control region based on the reference points
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           %%%%%%%%%%%%%%%%%%%%% CONTROL REGION 1 %%%%%%%%%%%%%%%%%%
           if (region == 0 && refValg < 0 && (( refH >= 0 && abs(refValg) >= tan(pi/6)*refH) || (refH < 0 && abs(refValg) >= tan(pi/6)*abs(refH)) ) ) % OK
                   % bla = 1;
                    region = 1;
                   %%% CHECKING if pressure in Ch1 is below minimum in order to start regulating position
                   if (check_pr_in_ch(1) == 0  && watchdog_Reg(1) < WATCHDOG_CONST) % if not, then deflate Ch1
                         pON = 0; pOFF = 1;  Ch1 = 1;  Ch2 = 0; Ch3 = 0;  % deflation of CH1
                         if (first_wait_run == 0)
                              waiting = 1;
                               checking_pr_in_ch = 1;
                              Saturated( (indAct-1)*3 + 1) = 0; %  if there is deflation, then there is no saturation OK
                         else 
                              if (pr < pr_low_limit)
                                   check_pr_in_ch(1) = 1;
                                   Ch1 = 0;  Ch2 = 0; Ch3 = 0; pON = 0; pOFF = 0; % everything OFF
                                   first_wait_run = 0;
                               checking_pr_in_ch = 0;
                              end
                       end
                   else % Pressure in Ch1 is  below minimum, START REGULATION
                          %%% VERTICAL REGULATION IN REGION 1 - regulation of Ch3
                          if (~H_or_V &&  vertical_regulation)  % I could have  started with horizontal, or ch 2
                               H_or_V = 1;
                               if (angleV -  V_offset_vect(indAct) > ( (angleH - refH)*tan(pi/6)  + refValg + 0.5*delta_angles) ) %  inflate Ch3 OK
                                        % inflate Ch3 or go to the //(changed 07.09.2016) next act if it's saturated in that chamber
                                        if ( Saturated( (indAct-1)*3 + 3) < MAX_SATUR_TRIALS)
                                              pON = 1; pOFF = 0;  Ch1 = 0;  Ch2 = 0; Ch3 = 1;
                                              check_pr_in_ch(3) = 0; 
                                              finshed_vert_reg = 0; % restart counter
                                              %watchdog_Reg(3) = 0;
                                         else % this chamber is saturated, change regulation direction
                                               vertical_regulation = 0;
                                               horiz_regulation = 1;
                                         end
                               elseif (angleV -  V_offset_vect(indAct) < ( (angleH - refH)*tan(pi/6)  + refValg - 0.5*delta_angles) )   %  deflate Ch3 OK
                                        pON = 0; pOFF = 1;  Ch1 = 0;  Ch2 = 0; Ch3 = 1;  %deflate Ch3
                                        Saturated( (indAct-1)*3 + 3) = 0;
                                        check_pr_in_ch(3) = 0; 
                                        finshed_vert_reg = 0; % restart counter
                                        if (pr < pr_low_limit)
                                              watchdog_Reg(1) = watchdog_Reg(1) + 1;                                                
                                        end
                                        if (watchdog_Reg(1) >= WATCHDOG_CONST)
                                              % inflate Ch1
                                              if ( Saturated( (indAct-1)*3 + 1) < MAX_SATUR_TRIALS)
                                                    Ch1 = 1;  Ch2 = 0; Ch3 = 0; pON = 1; pOFF = 0; 
                                              else
                                               vertical_regulation = 0;
                                               horiz_regulation = 1;
                                              end
                                               % inflate Ch1 (Ch1 in this region shouldn't be inflated, but this is an exception)
                                               %check_pr_in_ch(2) = 0; 
                                               % adding waiting = 1 ?
                                        end
                                else % angleV enough close to refV, everything off, exit this loop
                                       Ch1 = 0;  Ch2 = 0; Ch3 = 0; pON = 0; pOFF = 0;  % all off
                                        vertical_regulation = 0; 
                                        horiz_regulation = 1;
                                        finshed_vert_reg = finshed_vert_reg + 1;
                                        waiting_time = 0;
                                        watchdog_Reg(1) = 0;
                                 end
                           end % end of vertical regulation in control region 1
                           %%% HORIZONTAL REGULATION IN REGION 1 - regulation of Ch2
                            if (~H_or_V && horiz_regulation)
                                if (angleV -  V_offset_vect(indAct) > ( -(angleH - refH)*tan(pi/6)  + refValg + 0.5*delta_angles) ) %  inflate Ch2 OK
                                        % inflate Ch2
                                        if  ( Saturated( (indAct-1)*3 + 2) < MAX_SATUR_TRIALS)
                                                pON = 1; pOFF = 0;  Ch1 = 0;  Ch2 = 1; Ch3 = 0;
                                                check_pr_in_ch(2) = 0; 
                                                finshed_vert_reg = 0; % restart counter
                                                % watchdog_Reg(2) = 0;
                                       else 
                                               vertical_regulation = 1;
                                               horiz_regulation = 0;
                                       end
                                       
                                elseif (angleV -  V_offset_vect(indAct)  < ( -(angleH - refH)*tan(pi/6)  + refValg - 0.5*delta_angles) )   %  deflate Ch2 OK
                                        pON = 0; pOFF = 1;  Ch1 = 0;  Ch2 = 1; Ch3 = 0;  %deflate Ch2
                                        Saturated( (indAct-1)*3 + 2) = 0;
                                        check_pr_in_ch(2) = 0; 
                                        finshed_vert_reg = 0; % restart counter
                                        if (pr < pr_low_limit)
                                              watchdog_Reg(2) = watchdog_Reg(2) + 1;                                                
                                        end
                                        if (watchdog_Reg(2) >= WATCHDOG_CONST)
                                               % inflation of Ch1
                                               if  ( Saturated( (indAct-1)*3 + 1) < MAX_SATUR_TRIALS)
                                                        Ch1 = 0;  Ch2 = 0; Ch3 = 1; pON = 1; pOFF = 0;  % inflation of Ch1
                                              else
                                                    vertical_regulation = 1;
                                                    horiz_regulation = 0;
                                              end
                                               % inflate Ch3 (in this region Ch1 should be at zero pressure)
                                               check_pr_in_ch(3) = 0; 
                                        end
                                 else % angleV enough close to refV, everything off, exit this loop
                                        Ch1 = 0;  Ch2 = 0; Ch3 = 0; pON = 0; pOFF = 0;  % all off
                                        vertical_regulation = 1; 
                                        horiz_regulation = 0;
                                        finshed_horiz_reg = finshed_horiz_reg + 1;
                                        waiting_time = 0;
                                               watchdog_Reg(2) = 0;
                                 end                         
                         end % end of horizontal regulation in this control region
                    end % end of check pr in ch 1 and regulation
            end % end of checking if the reference point is within Region 1
 
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           %%%%%%%%%%%%%%%%%%%%% CONTROL REGION 2 %%%%%%%%%%%%%%%%%%
           %if (refH >= 0 && (refV >= 0 || refV < 0 && -refV > - tan(pi/6)*refH ) ) % OK
            if (region == 0 && refH >= 0 && (refValg >= 0 || refValg < 0 && abs(refValg) < tan(pi/6)*refH ) )
                    % bla = 1;
                    region = 2;
                   %%% Checking if pressure in Ch2 is below minimum in order to start regulating position
                   if (check_pr_in_ch(2) == 0 && watchdog_Reg(3) < WATCHDOG_CONST) % if not, then deflate Ch2
                          pON = 0; pOFF = 1;  Ch1 = 0;  Ch2 = 1; Ch3 = 0;  % deflate Ch2
                          if (first_wait_run ==0)
                               waiting = 1;  
                               checking_pr_in_ch = 1;
                               Saturated( (indAct-1)*3 + 2) = 0;
                          else          
                                if (pr < pr_low_limit)
                                    check_pr_in_ch(2) = 1;
                                    Ch1 = 0;  Ch2 = 0; Ch3 = 0; pON = 0; pOFF = 0; % everything OFF
                                    first_wait_run = 0;  
                                    checking_pr_in_ch = 0;
                               end
                         end
                   else % Pressure in Ch2 is  below minimum, START REGULATION
                         %%% VERTICAL REGULATION IN REGION 2 - regulation of Ch1
                          if (~H_or_V && vertical_regulation)  % I could have  started with horizontal regulation
                               H_or_V = 1;
                               if (angleV -  V_offset_vect(indAct) < ( -(angleH - refH)*tan(pi/6)  + refValg - 0.5*delta_angles) ) %  inflate Ch1 OK
                                        ledX = 200;
                                        % inflate Ch1
                                         if (Saturated( (indAct-1)*3 + 1) < MAX_SATUR_TRIALS)
                                                 pON = 1; pOFF = 0;  Ch1 = 1;  Ch2 = 0; Ch3 = 0;
                                         else
                                               vertical_regulation = 0;
                                               horiz_regulation = 1; 
                                         end
                                        check_pr_in_ch(1) = 0; 
                                        finshed_vert_reg = 0; % restart counter
                                               %watchdog_Reg(1) = 0;
                               elseif (angleV -  V_offset_vect(indAct) > ( -(angleH - refH)*tan(pi/6)  + refValg + 0.5*delta_angles) )   %  deflate Ch1 OK
                                        pON = 0; pOFF = 1;  Ch1 = 1;  Ch2 = 0; Ch3 = 0;  % deflate Ch1
                                        Saturated( (indAct-1)*3 + 1) = 0;
                                        check_pr_in_ch(1) = 0; 
                                        finshed_vert_reg = 0; % restart counter
                                        if (pr < pr_low_limit)
                                              watchdog_Reg(3) = watchdog_Reg(3) + 1;                                                
                                        end
                                        if (watchdog_Reg(3) >= WATCHDOG_CONST)
                                                % inflate ch2
                                                 if (Saturated( (indAct-1)*3 + 2) < MAX_SATUR_TRIALS)
                                                         Ch1 = 0;  Ch2 = 1; Ch3 = 0; pON = 1; pOFF = 0; 
                                              % inflate Ch2 (exactly this one that should be at zero, but this is an exception)
                                              %check_pr_in_ch(2) = 0; 
                                                 else 
                                                       vertical_regulation = 0;
                                                       horiz_regulation = 1; 
                                                 end
                                       end
                                else % angleV enough close to refV, everything off, exit this loop
                                       Ch1 = 0;  Ch2 = 0; Ch3 = 0; pON = 0; pOFF = 0;  % all off
                                        vertical_regulation = 0;                     
                                        horiz_regulation = 1;
                                        finshed_vert_reg = finshed_vert_reg + 1;
                                        waiting_time = 0;
                                        watchdog_Reg(3) = 0;
                                 end
                           end % end of vertical regulation in control region 2
                           %%% HORIZONTAL REGULATION IN REGION 2 - regulation of Ch3
                            if ( ~H_or_V &&  horiz_regulation)
                                if (angleH < refH - 0.5*delta_angles) 
                                        %  inflate Ch3 OK
                                        if (Saturated( (indAct-1)*3 + 3) < MAX_SATUR_TRIALS)
                                            pON = 1; pOFF = 0;  Ch1 = 0;  Ch2 = 0; Ch3 = 1;
                                            check_pr_in_ch(3) = 0; 
                                             finshed_vert_reg = 0; % restart counter
                                               %watchdog_Reg(3) = 0;
                                       else
                                              vertical_regulation = 1;
                                               horiz_regulation = 0;                                            
                                       end
                                elseif (angleH > refH + 0.5*delta_angles)   %  deflate Ch3 OK
                                        pON = 0; pOFF = 1;  Ch1 = 0;  Ch2 = 0; Ch3 = 1;  % deflate Ch3
                                        Saturated( (indAct-1)*3 + 3) = 0;
                                        check_pr_in_ch(3) = 0; 
                                        finshed_vert_reg = 0; % restart counter
                                        if (pr < pr_low_limit)
                                              watchdog_Reg(4) = watchdog_Reg(4) + 1;                                                
                                        end
                                        if (watchdog_Reg(4) >= WATCHDOG_CONST)
                                                % inflate Ch1
                                                if (Saturated( (indAct-1)*3 + 1)  < MAX_SATUR_TRIALS)
                                                      Ch1 = 1;  Ch2 = 0; Ch3 = 0; pON = 1; pOFF = 0; 
                                                     % inflate Ch1 (in this region Ch2 should be at zero pressure)
                                                       check_pr_in_ch(1) = 0; 
                                                 else
                                                      vertical_regulation = 1;
                                                     horiz_regulation = 0;                      
                                                  end
                                        end
                                 else % angleV enough close to refV, everything off, exit this loop
                                        Ch1 = 0;  Ch2 = 0; Ch3 = 0; pON = 0; pOFF = 0;  % all off
                                        vertical_regulation = 1; 
                                        horiz_regulation = 0;
                                        finshed_horiz_reg = finshed_horiz_reg + 1;
                                        waiting_time = 0;
                                               watchdog_Reg(4) = 0;
                                 end                         
                         end % end of horizontal regulation in this control region
                    end % end of check pr in ch 2 and regulation
            end % end of checking if the reference point is within Region 2

           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           %%%%%%%%%%%%%%%%%%%%% CONTROL REGION 3 %%%%%%%%%%%%%%%%%%
            if (region == 0 &&  refH < 0 && ( refValg >= 0 || refValg < 0 && refValg > tan(pi/6) *refH ) ) % OK
                   %  bla = 1;
                    ledX = 4;
                    region = 3;
                   %%% Checking if pressure in Ch3 is below minimum in order to start regulating position
                   if (check_pr_in_ch(3) == 0  && watchdog_Reg(5) < WATCHDOG_CONST ) % if not, then deflate Ch3
                         pON = 0; pOFF = 1;  Ch1 = 0;  Ch2 = 0; Ch3 = 1;  % deflation of Ch3
                         if (first_wait_run == 0)
                                 waiting = 1; 
                                 checking_pr_in_ch = 1;
                                 Saturated( (indAct-1)*3 + 3) = 0;
                         else
                                if (pr < pr_low_limit)
                                      check_pr_in_ch(3) = 1;
                                      Ch1 = 0;  Ch2 = 0; Ch3 = 0; pON = 0; pOFF = 0; % everything OFF
                                       first_wait_run =0; 
                                       checking_pr_in_ch = 0;
                               end
                         end
                   else % Pressure in Ch3 is  below minimum, START REGULATION
                         %%% VERTICAL REGULATION IN REGION 3 - regulation of Ch2
                          if (~H_or_V &&  vertical_regulation)  % I could have  started with horizontal regulation
                                H_or_V = 1;
                               if (angleH > refH + 0.5*delta_angles ) 
                                       %  inflate Ch2 OK
                                       if (  Saturated( (indAct-1)*3 + 2)  < MAX_SATUR_TRIALS)
                                              pON = 1; pOFF = 0;  Ch1 = 0;  Ch2 = 1; Ch3 = 0;
                                              check_pr_in_ch(2) = 0; 
                                               finshed_vert_reg = 0; % restart counter
                                              % watchdog_Reg(2) = 0;
                                      else
                                              vertical_regulation = 0;
                                               horiz_regulation = 1;                      
                                       end
                               elseif (angleH < refH - 0.5*delta_angles )   %  deflate Ch2 OK
                                        pON = 0; pOFF = 1;  Ch1 = 0;  Ch2 = 1; Ch3 = 0;  % deflate Ch2
                                        Saturated( (indAct-1)*3 + 2) = 0;
                                        check_pr_in_ch(2) = 0; 
                                        finshed_vert_reg = 0; % restart counter
                                        if (pr < pr_low_limit)
                                              watchdog_Reg(5) = watchdog_Reg(5) + 1;
                                        end
                                        if (watchdog_Reg(5) >= WATCHDOG_CONST)
                                             % inflate Ch3
                                             if (Saturated( (indAct-1)*3 + 3)  < MAX_SATUR_TRIALS)
                                                    Ch1 = 0;  Ch2 = 0; Ch3 = 1; pON = 1; pOFF = 0; 
                                                    % inflate Ch3 (normally shouldn't be inflated in this region, but this is an exception)
                                                     % check_pr_in_ch(1) = 0; 
                                             else
                                                    vertical_regulation = 0;
                                                    horiz_regulation = 1;    
                                             end
                                        end
                                else % angleV enough close to refV, everything off, exit this loop
                                       Ch1 = 0;  Ch2 = 0; Ch3 = 0; pON = 0; pOFF = 0;  % all off
                                        vertical_regulation = 0; 
                                        horiz_regulation = 1;
                                         finshed_vert_reg = finshed_vert_reg + 1;
                                         waiting_time = 0;
                                               watchdog_Reg(5) = 0;
                                 end
                           end % end of vertical regulation in control region 3
                           %%% HORIZONTAL REGULATION IN REGION 3 - regulation of Ch1
                            if (~H_or_V &&  horiz_regulation)
                                if (angleV -  V_offset_vect(indAct) < ( (angleH - refH)* tan(pi/6)  + refValg - 0.5*delta_angles* tan(pi/6))) %  inflate Ch1 OK
                                         % inflate Ch 1
                                         if ( Saturated( (indAct-1)*3 + 1)  < MAX_SATUR_TRIALS)
                                              pON = 1; pOFF = 0;  Ch1 = 1;  Ch2 = 0; Ch3 = 0;
                                              check_pr_in_ch(1) = 0; 
                                               finshed_vert_reg = 0; % restart counter
                                               %watchdog_Reg(1) = 0;
                                         else
                                               vertical_regulation = 1;
                                               horiz_regulation = 0;    
                                         end
                                elseif (angleV -  V_offset_vect(indAct) > ( (angleH - refH)* tan(pi/6)  + refValg + 0.5*delta_angles* tan(pi/6))) %  deflate Ch1 OK
                                        pON = 0; pOFF = 1;  Ch1 = 1;  Ch2 = 0; Ch3 = 0;  % deflate Ch1
                                       Saturated( (indAct-1)*3 + 1) = 0;
                                        check_pr_in_ch(1) = 0; 
                                        finshed_vert_reg = 0; % restart counter
                                        if (pr < pr_low_limit)
                                              watchdog_Reg(6) = watchdog_Reg(6) + 1;                                                
                                        end
                                        if (watchdog_Reg(6) >= WATCHDOG_CONST)
                                                % inflate Ch2
                                                if (Saturated( (indAct-1)*3 + 2)  < MAX_SATUR_TRIALS)
                                                     Ch1 = 0;  Ch2 = 1; Ch3 = 0; pON = 1; pOFF = 0; 
                                                    % inflate Ch2 (in this region Ch3 should be at zero pressure)
                                                      check_pr_in_ch(2) = 0; 
                                                else
                                                       vertical_regulation = 1;
                                                       horiz_regulation = 0; 
                                                end
                                        end
                                 else % angleV enough close to refV, everything off, exit this loop
                                        Ch1 = 0;  Ch2 = 0; Ch3 = 0; pON = 0; pOFF = 0;  % all off
                                        vertical_regulation = 1; 
                                        horiz_regulation = 0;
                                        finshed_horiz_reg = finshed_horiz_reg + 1;
                                        waiting_time = 0;
                                               watchdog_Reg(6) = 0;
                                 end                         
                         end % end of horizontal regulation in Control Region 3
                    end % end of check pr in ch 3 and regulation
            end % end of checking if the reference point is within Region 3

             % SAFETY MECHANISMS (PRESSURE LIMIT) FOR THE ACTUATORS
             %led1 = 0;
             temp1=0;
             %if (~exit_Saturation)
             if (pr >= maxPr( (indAct-1)*3 + 1)  && Ch1 == 1 && pON == 1)
                   Ch1 = 0;
                  temp1 = 1;
                  Saturated((indAct-1)*3 + 1) = Saturated((indAct-1)*3 + 1)+ 1;
             elseif (pr >= maxPr( (indAct-1)*3 + 2)    && Ch2 == 1 && pON == 1)
                   Ch2 = 0; 
                   temp1 = 1;
                  Saturated((indAct-1)*3 + 2) = Saturated((indAct-1)*3 + 2) + 1;
             elseif (pr >= maxPr( (indAct-1)*3 + 3)   && Ch3 == 1 && pON == 1)
                   Ch3 = 0;  
                   temp1 = 1;
                  Saturated((indAct-1)*3 + 3) = Saturated((indAct-1)*3 + 3)+ 1;
             end
             if temp1
                 first_run_COM_pr_checked = 0;
                 %ledX = 300;
                 pON=0; pOFF =0; 
                 % Ch1 = 0; Ch2 = 0; Ch3 = 0;
                 %horiz_regulation = 0;
                 %vertical_regulation = 0;
                 %regulation_ongoing = 0;
                 % commented 07.09. not to jump after saturating one chamber - nextAct = 1;
                 if (horiz_regulation == 1)
                         horiz_regulation = 0;
                         vertical_regulation = 1;
                 elseif (vertical_regulation == 1)
                         horiz_regulation =1;
                         vertical_regulation = 0;
                end
             end

             % if actuator is outside of allowed limit, deflate what the algorithm was trying ot inflatean and use other regulation
             if (angleV >  refV_MAX || angleV <  refV_MIN || angleH >  refH_MAX || angleH <  refH_MIN )
                       out_of_limits = out_of_limits + 1;
                       % start deflating
                       pON=0; pOFF =1;
                       if (DELTA_ANGLE_ADDITIONAL_LIMIT >= 100)
                                Ch1 = 1;  Ch2 =1; Ch3 = 1; 
                       end
                       waiting = 1; 
                       if (out_of_limits >=2)
                              if (horiz_regulation == 1)
                                      horiz_regulation = 0;
                                      vertical_regulation = 1;
                              elseif (vertical_regulation == 1)
                                      horiz_regulation =1;
                                      vertical_regulation = 0;
                              end
                      elseif (out_of_limits >=5)
                             nextAct = 1;
                       end
              end
    

             % CHECKING THE VALIDITY OF THE SENSORS
             %validityMat = [7, 8, 5, 6, 3, 4, 1, 2];
             %indSen = validityMat(indAct);
             CurrentSignalValidity = 0;
             if (indAct <= 8) 
                  CurrentSignalValidity = IMU_Validity2(indAct); % IMU sensor kit 2 (send RaspPi)
             else
                  CurrentSignalValidity = IMU_Validity1(indAct-8); % IMU sensor kit 1 (first RaspPi)
             end
             if (~CurrentSignalValidity)
                   Ch1 = 0;  Ch2 = 0; Ch3 = 0; pON = 0; pOFF = 0; 
                   nextAct = 1;
             end
        end % of checking if the actuator is saturated
        end % OF REGULATION IN CONTROL REGIONS
        % of  if, else (~regulation_ongoing || refV_angle_old ~= refV_angle || refH_angle_old ~= refH_angle )

       %%% Exit code from this loop - if angle regulation was 2 times checked, stop cycling horizontal-vertical regulation
       if (finshed_vert_reg >= 2 && finshed_horiz_reg >= 2 )
             finshed_vert_reg = 0;
             finshed_horiz_reg = 0;
             vertical_regulation = 0;
             horiz_regulation = 0;
             regulation_ongoing = 0;
             % This is the place where I can change the way the control works
             % Either sequential after small steps or sequential after the final orientation has been achieved
             if ( abs(angleV - refV_FINAL) <= 0.5*delta_angles && ...  % THIS LINE
                  abs(angleH - refH_FINAL) <= 0.5*delta_angles)            % THIS LINE
                        nextAct = 1;
%%                       indAct = indAct + 1;
%%                       if indAct > N_Modules
  %%                          indAct = 1;
   %%                    end
                         % first_run_COM_pr_checked = 0;
             end                                                                                           % THIS LINE
             %first_run_COM_pr_checked = 0;
             %inter_ref_valid = 0;
      end % of  if (finshed_vert_reg >= 2 && finshed_horiz_reg >= 2 )

end % of if (~first_run_com_pr_check)      
end % of waiting
end % of if (stop_regulation)

% forming outDO_vector
Chs = zeros(1,N_Modules*3);
Chs(1, (indAct-1)*3+1: (indAct-1)*3+3) =  [Ch1, Ch2, Ch3];
out_DO_vector  = [pON, pOFF, Chs];

Pref = 0;

% forming state_out
refV_angle_old = refV_FINAL;
refH_angle_old = refH_FINAL;
state_out =  [check_pr_in_ch waiting waiting_time pON pOFF ...
     horiz_regulation vertical_regulation ...
      finshed_vert_reg finshed_horiz_reg regulation_ongoing ...
     watchdog_Reg specialR delta_deg_in_V_reached ...
     delta_deg_in_H_reached inter_ref_valid refV refH refV_angle_old...
     refH_angle_old  exit_Saturation indAct  flag_MActFinished ...
     first_run_COM_pr_checked nextAct refChanged SunTrSeq_old ...
     first_wait_run checking_pr_in_ch  out_of_limits Saturated  Chs];
