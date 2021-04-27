%{
%AER403 - Mechanism & Vibrations - W2021 - Final Project

Khaled Hazzam - 46540
Nolan Robins - 52814
Marcin Pawluch - 75631

THIS FILE REQUIRES link.m & pivot_link.m TO FUNCTION, FAILURE TO HAVE
THESE FILES IN THE SAME DIRECTORY OF PATH WILL RESULT IN FAILURE TO OPERATE

Simulation starts in a stopped state, started with the press of the start
button, the simulation will also automatically render all four legs when
they are avaliable, legs 2 and 4 render after a 180 degree revolution, to
turn these legs off three buttons are avaliable on the right side of the
figure screen.

Velocity vectors and the path of the foot can be seen by pressing their
respective buttons on the left side of the figure screen.

%}

%%Reset Matlab Memory
close all
clear
clc
%%
%Set Default RPM and Gear Ratio
rpm = 8000;
gear_ratio = 5402;

total_rotations = 12;
resolution_per_rotation = 100;


%%
%Create Figure and all buttons/controls of the ui
mainfig = figure('Name','Simulation', 'NumberTitle','off'); 
sim_start = uicontrol('Parent',mainfig,'Style','pushbutton','String','Start','Units','normalized', 'Position',[0.01 0.07 .05 .05], 'callback', 'sim_state = 1;');
sim_pause = uicontrol('Parent',mainfig,'Style','pushbutton','String','Pause','Units','normalized', 'Position',[0.01 0.01 .05 .05], 'callback', 'sim_state = 0;');
draw_velocity = uicontrol('Parent',mainfig,'Style','pushbutton','String','Draw Velocity','Units','normalized', 'Position',[0.01 0.13 .05 .05], 'callback', 'chng_velocity_draw = 1;');
draw_circle = uicontrol('Parent',mainfig,'Style','pushbutton','String','Draw Intersection','Units','normalized', 'Position',[0.01 0.19 .05 .05], 'callback', 'chng_circle_draw = 1;');
draw_foot = uicontrol('Parent',mainfig,'Style','pushbutton','String','Draw Path','Units','normalized', 'Position',[0.01 0.25 .05 .05], 'callback', 'chng_foot_draw = 1;');
rpm_set = uicontrol('Parent',mainfig,'Style','edit','String',rpm,'Units','normalized', 'Position',[0.01 0.31 .05 .03]);
uicontrol('Parent',mainfig,'Style','text','String','RPM:','Units','normalized', 'Position',[0.01 0.34 .05 .025]);
gear_ratio_set = uicontrol('Parent',mainfig,'Style','edit','String',gear_ratio,'Units','normalized', 'Position',[0.01 0.38 .05 .03]);
uicontrol('Parent',mainfig,'Style','text','String','Gear Ratio:','Units','normalized', 'Position',[0.01 0.42 .05 .025]);
draw_leg_2 = uicontrol('Parent',mainfig,'Style','pushbutton','String','Draw Leg 2','Units','normalized', 'Position',[0.94 0.01 .05 .05], 'callback', 'chng_leg_2 = 1;');
draw_leg_3 = uicontrol('Parent',mainfig,'Style','pushbutton','String','Draw Leg 3','Units','normalized', 'Position',[0.94 0.07 .05 .05], 'callback', 'chng_leg_3 = 1;');
draw_leg_4 = uicontrol('Parent',mainfig,'Style','pushbutton','String','Draw Leg 4','Units','normalized', 'Position',[0.94 0.13 .05 .05], 'callback', 'chng_leg_4 = 1;');

axis([-100 , 100, -100, 100]);

%%
%Set points for main mechanism, these can be varied to allow for different
%configurations of this same design but an error of "Intersection Error" in
%the console means the points are invalid and cannot work.
motor_rotation_point = [0, 0];
link_input_point = [0, 15];
joint_bottom_point = [-20, -10];
joint_bottom_connection_point = [-20, 13];
joint_top_point = [5, 15];
joint_top_connection_point = [-12, 30];
leg_mid_point = [-30, 0];
leg_bottom_point = [-30, -40];

%MAKE MAIN JOINT

%Calculate the length of some of the joints in order to feed to subsequent
%functions in order to keep constant radius
pivot_bottom_radius = sqrt((joint_bottom_point(1) - joint_bottom_connection_point(1))^2+(joint_bottom_point(2) - joint_bottom_connection_point(2))^2);
pivot_top_radius = sqrt((joint_top_point(1) - joint_top_connection_point(1))^2 + (joint_top_point(2) - joint_top_connection_point(2))^2);

leg_motor_link_length = sqrt((joint_bottom_connection_point(1) - link_input_point(1))^2 + (joint_bottom_connection_point(2) - link_input_point(2))^2);
leg_motor_link_length_output = sqrt((joint_top_connection_point(1) - leg_mid_point(1))^2 + (joint_top_connection_point(2) - leg_mid_point(2))^2);

%Create all linkages of the body, including input motor arm, and pivot arms
motor_arm = link([motor_rotation_point; link_input_point]);
leg_motor_link = link([link_input_point; joint_bottom_connection_point; leg_mid_point]);
leg_link = link([leg_mid_point; joint_top_connection_point; leg_bottom_point]);
pivot_arm_bottom = pivot_link(joint_bottom_point, pivot_bottom_radius);
pivot_arm_top = pivot_link(joint_top_point, pivot_top_radius);

%Construct linkage assembly by placing items together
motor_arm.set_input_joint(1,0);
motor_arm.set_input_function("rotation",0,[0,pi]);
motor_arm.set_output_joints(2,leg_motor_link);

leg_motor_link.set_input_joint(1, motor_arm);
leg_motor_link.set_pivot_links(2, pivot_arm_bottom);
leg_motor_link.set_output_joints(3, leg_link);

leg_link.set_input_joint(1, leg_motor_link);
leg_link.set_pivot_links(2, pivot_arm_top);
leg_link.set_output_joints(3, 0);

%%
%Build Opposite leg in the same way as before
link_input_point_3 = link_input_point;
joint_bottom_point_3 = [20, -10];
joint_bottom_connection_point_3 = [20, 13];
joint_top_point_3 = [-5, 15];
joint_top_connection_point_3 = [12, 30];
leg_mid_point_3 = [30, 0];
leg_bottom_point_3 = [30, -40];

motor_arm_3 = link([motor_rotation_point; link_input_point]);
leg_motor_link_3 = link([link_input_point; joint_bottom_connection_point_3; leg_mid_point_3]);
leg_link_3 = link([leg_mid_point_3; joint_top_connection_point_3; leg_bottom_point_3]);
pivot_arm_bottom_3 = pivot_link(joint_bottom_point_3, pivot_bottom_radius);
pivot_arm_top_3 = pivot_link(joint_top_point_3, pivot_top_radius);


motor_arm_3.set_input_joint(1,0);
motor_arm_3.set_input_function("rotation",0,[0,pi]);
motor_arm_3.set_output_joints(2,leg_motor_link_3);

leg_motor_link_3.set_input_joint(1, motor_arm_3);
leg_motor_link_3.set_pivot_links(2, pivot_arm_bottom_3);
leg_motor_link_3.set_output_joints(3, leg_link_3);

leg_link_3.set_input_joint(1, leg_motor_link_3);
leg_link_3.set_pivot_links(2, pivot_arm_top_3);
leg_link_3.set_output_joints(3, 0);


%%
%Set various state variables to keep track of buttons pressed on the menu
%and allow for their usage
sim_state = 0;
velocity_draw = 0;
circle_draw = 0;
foot_draw = 0;
leg_2_draw = 1;
leg_3_draw = 1;
leg_4_draw = 1;
chng_velocity_draw = 0;
chng_circle_draw = 0;
chng_foot_draw = 0;
chng_leg_2 = 0;
chng_leg_3 = 0;
chng_leg_4 = 0;

%Store bottom foot position in order to output later
foot_pos = [];
foot_pos_2 = [];

%%
%Main loop
for i = 1:total_rotations*resolution_per_rotation
    
    %Get textbox data and update
    rpm = str2double(rpm_set.String);
    gear_ratio = str2double(gear_ratio_set.String);
    
    %calculate rotations per second after gearbox
    rps = rpm*gear_ratio/60;
    
    %wait if in pause state
    while sim_state == 0
        pause(0.1)
    end
    
    %Build main Subplot
    subplot(1,1,1)
    
    %Send update command to pivot arms to find new pivot position
    pivot_arm_bottom.update_point(leg_motor_link.joint_points(1,:), leg_motor_link_length, 0);
    pivot_arm_top.update_point(leg_motor_link.joint_points(3,:), leg_motor_link_length_output, 1);
    
    %Send update command to all other links to begin building
    motor_arm.update_now(1,resolution_per_rotation, 0)
    leg_motor_link.update_now(1,resolution_per_rotation,0)
    leg_link.update_now(1,resolution_per_rotation,0)
    
    %Draw all parts of leg one
    pivot_arm_bottom.draw_now();
    
    hold on
    axis([-100 , 100, -100, 100]);
    axis square;
    
    pivot_arm_top.draw_now();
    
    motor_arm.draw_now('red')
    leg_motor_link.draw_now('blue')
    leg_link.draw_now('green')
    
    %Update all points of leg 3, even if not being drawn
    pivot_arm_bottom_3.update_point(leg_motor_link_3.joint_points(1,:), leg_motor_link_length, 0);
    pivot_arm_top_3.update_point(leg_motor_link_3.joint_points(3,:), leg_motor_link_length_output, 1);
    
    motor_arm_3.update_now(1,resolution_per_rotation, 0)
    leg_motor_link_3.update_now(1,resolution_per_rotation,0)
    leg_link_3.update_now(1,resolution_per_rotation,0)
    
    %If leg 3 is being drawn, draw now
    if leg_3_draw
        pivot_arm_bottom_3.draw_now();
        pivot_arm_top_3.draw_now();
        motor_arm_3.draw_now('red')
        leg_motor_link_3.draw_now('blue')
        leg_link_3.draw_now('green')
    end
    
    %%
    %Check state of button and update accordingly
    if chng_velocity_draw
        chng_velocity_draw = 0;
        velocity_draw = ~velocity_draw;
    end
    
    
    if chng_circle_draw
        chng_circle_draw = 0;
        circle_draw = ~circle_draw;
    end
    
    if chng_foot_draw
        chng_foot_draw = 0;
        foot_draw = ~foot_draw;
    end
    
    if chng_leg_2
        chng_leg_2 = 0;
        leg_2_draw = ~leg_2_draw;
    end
    
    if chng_leg_3
        chng_leg_3 = 0;
        leg_3_draw = ~leg_3_draw;
    end
    
    if chng_leg_4
        chng_leg_4 = 0;
        leg_4_draw = ~leg_4_draw;
    end
    
    %If velocity is to be drawn, send command to all avaliable links that
    %are currently being drawn.
    if velocity_draw
        leg_link.draw_velocity(resolution_per_rotation/rps*1000);
        leg_motor_link.draw_velocity(resolution_per_rotation/rps*1000);
        if leg_2_draw && i > resolution_per_rotation/2
            leg_link_2.draw_velocity(resolution_per_rotation/rps*1000);
            leg_motor_link_2.draw_velocity(resolution_per_rotation/rps*1000);
        end
        if leg_3_draw
            leg_link_3.draw_velocity(resolution_per_rotation/rps*1000);
            leg_motor_link_3.draw_velocity(resolution_per_rotation/rps*1000);
        end
        if leg_4_draw && i > resolution_per_rotation/2
            leg_link_4.draw_velocity(resolution_per_rotation/rps*1000);
            leg_motor_link_4.draw_velocity(resolution_per_rotation/rps*1000);
        end
    end
    
    %If intersections are to be drawn do it now, this is mostly to ensure
    %the answer is correct as each point after a pivot must have its
    %respective two circles intersect at ALL TIMES, if these circles fail
    %to intersect the solution is impossible and the mechanism not valid.
    if circle_draw
        pivot_arm_top.draw_circles(leg_motor_link.joint_points(3,:),leg_motor_link_length_output);
        pivot_arm_bottom.draw_circles(leg_motor_link.joint_points(1,:), leg_motor_link_length);
    end
    
    %Draw path of feet, the i > 25 is due to a bug of the initial state of
    %leg 3
    if foot_draw
        plot(foot_pos(:,1),foot_pos(:,2));
        if (leg_3_draw || leg_4_draw) && i > 25
            plot(foot_pos_2(:,1),foot_pos_2(:,2));
        end
    end    
    
    %Store foot coordinate for future display, this is done after 24 due to
    %bug
    foot_pos = [foot_pos ; leg_link.joint_points(3,:)];
    if i > 24
        foot_pos_2 = [foot_pos_2 ; leg_link_3.joint_points(3,:)];
    end
       
    %%
    %If first leg has gone through 180degrees of rotation, draw new leg at
    %beginning position 180degrees out of phase, this acts as the leg
    %attached to the opposite side of the motor, the construction of this
    %leg is nearly identical to the first leg, reference to this structure
    %can be viewed above.
    if i == resolution_per_rotation/2
        
        link_input_point_2 = link_input_point;
        joint_bottom_connection_point_2 = joint_bottom_connection_point;
        joint_top_connection_point_2 = joint_top_connection_point;
        leg_mid_point_2 = leg_mid_point;
        leg_bottom_point_2 = leg_bottom_point;
        
        
        motor_arm_2 = link([motor_rotation_point; link_input_point_2]);
        leg_motor_link_2 = link([link_input_point_2; joint_bottom_connection_point_2; leg_mid_point_2]);
        leg_link_2 = link([leg_mid_point_2; joint_top_connection_point_2; leg_bottom_point_2]);
        pivot_arm_bottom_2 = pivot_link(joint_bottom_point, pivot_bottom_radius);
        pivot_arm_top_2 = pivot_link(joint_top_point, pivot_top_radius);
        
        
        motor_arm_2.set_input_joint(1,0);
        motor_arm_2.set_input_function("rotation",0,[0,pi]);
        motor_arm_2.set_output_joints(2,leg_motor_link_2);
        
        leg_motor_link_2.set_input_joint(1, motor_arm_2);
        leg_motor_link_2.set_pivot_links(2, pivot_arm_bottom_2);
        leg_motor_link_2.set_output_joints(3, leg_link_2);
        
        leg_link_2.set_input_joint(1, leg_motor_link_2);
        leg_link_2.set_pivot_links(2, pivot_arm_top_2);
        leg_link_2.set_output_joints(3, 0);
        
        %%
        
        link_input_point_4 = link_input_point;
        joint_bottom_point_4 = [20, -10];
        joint_bottom_connection_point_4 = [20, 13];
        joint_top_point_4 = [-5, 15];
        joint_top_connection_point_4 = [12, 30];
        leg_mid_point_4 = [30, 0];
        leg_bottom_point_4 = [30, -40];
        
        
        motor_arm_4 = link([motor_rotation_point; link_input_point]);
        leg_motor_link_4 = link([link_input_point; joint_bottom_connection_point_4; leg_mid_point_4]);
        leg_link_4 = link([leg_mid_point_4; joint_top_connection_point_4; leg_bottom_point_4]);
        pivot_arm_bottom_4 = pivot_link(joint_bottom_point_3, pivot_bottom_radius);
        pivot_arm_top_4 = pivot_link(joint_top_point_3, pivot_top_radius);
        
        
        motor_arm_4.set_input_joint(1,0);
        motor_arm_4.set_input_function("rotation",0,[0,pi]);
        motor_arm_4.set_output_joints(2,leg_motor_link_4);
        
        leg_motor_link_4.set_input_joint(1, motor_arm_4);
        leg_motor_link_4.set_pivot_links(2, pivot_arm_bottom_4);
        leg_motor_link_4.set_output_joints(3, leg_link_4);
        
        leg_link_4.set_input_joint(1, leg_motor_link_4);
        leg_link_4.set_pivot_links(2, pivot_arm_top_4);
        leg_link_4.set_output_joints(3, 0);
        
    end
    
    %If the 180degree out of phase legs have been drawn, beginning updating
    %and check for whether to display or not.
    if i > resolution_per_rotation/2 
        pivot_arm_bottom_2.update_point(leg_motor_link_2.joint_points(1,:), leg_motor_link_length, 0);
        pivot_arm_top_2.update_point(leg_motor_link_2.joint_points(3,:), leg_motor_link_length_output, 1);
    
        motor_arm_2.update_now(1,resolution_per_rotation, 0)
        leg_motor_link_2.update_now(1,resolution_per_rotation,0)
        leg_link_2.update_now(1,resolution_per_rotation,0)
        
        pivot_arm_bottom_4.update_point(leg_motor_link_4.joint_points(1,:), leg_motor_link_length, 0);
        pivot_arm_top_4.update_point(leg_motor_link_4.joint_points(3,:), leg_motor_link_length_output, 1);
    
        motor_arm_4.update_now(1,resolution_per_rotation, 0)
        leg_motor_link_4.update_now(1,resolution_per_rotation,0)
        leg_link_4.update_now(1,resolution_per_rotation,0)
        
        if leg_2_draw
            pivot_arm_bottom_2.draw_now();
            
            pivot_arm_top_2.draw_now();
        
            motor_arm_2.draw_now('red')
            leg_motor_link_2.draw_now('blue')
            leg_link_2.draw_now('green')
        end
        
        if leg_4_draw
            pivot_arm_bottom_4.draw_now();
            
            pivot_arm_top_4.draw_now();
        
            motor_arm_4.draw_now('red')
            leg_motor_link_4.draw_now('blue')
            leg_link_4.draw_now('green')
        end
    end
    
    hold off
    
    pause(.05)
    
end

