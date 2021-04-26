classdef link < handle
    % Constructor for main linkages
    %   This class should be able to create any linkage system by combining
    %   these classes together
    % Joint_structure should be 1 - input, 2 - pivot, 3 - output
    % Created by Nolan Robins
    % Version 0.5
    
    properties
        number_of_joints = 0
        joint_points
        anchor_joints
        input_joints
        input_link
        pivot_joints
        pivot_links
        output_joints
        output_links
        old_output_point = [0, 0]
        input_type = 0
        input_limits = 0
        current_direction
    end
    
    methods
        function obj = link(joint_points)
            [obj.number_of_joints, rm] = size(joint_points);
            obj.joint_points = joint_points;
        end
        
        function [theta , rho] = get_vector(obj, joint_1, joint_2)
            %Gets the vector between two joint points
            vector_cartesian = obj.joint_points(joint_2,:) - obj.joint_points(joint_1,:);
            [theta, rho] = cart2pol(vector_cartesian(1), vector_cartesian(2));
        end
        
        function obj = set_anchor(obj, joint_number)
            %Sets anchor, this has no purpose, i think, im too scared to delete it. it can be
            %considered depricated though it may serve a purpose in the
            %future??? this same functionality can be done with a pivot
            %joint of near 0 length, though im gonna leave this here
            obj.anchor_joints = joint_number;
        end
        
        function obj = set_input_joint(obj, joint_number, link)
            %Sets which joint is the input joint and ties the link data
            obj.input_joints = joint_number;
            obj.input_link = link;
        end
        
        function obj = set_output_joints(obj, joint_numbers, links)
            %Sets which joint is the output joint and ties the link data
            obj.output_joints = joint_numbers;
            obj.output_links = links;
        end
        
        function obj = set_pivot_links(obj, joint_numbers, links)
            %Sets which joint is the pivot and ties the pivot_link data
            obj.pivot_joints = joint_numbers;
            obj.pivot_links = links;
        end
        
        function obj = set_input_function(obj, function_input_type, axis, limits)
            %Input types for main input linkage, support for rotation is
            %currently included, functionality may be expanded here
            %1 = rotational, 2 = linear
            %axis for rotational does not matter, axis for rectangular is
            %the axis on which it moves along.
            %Limits should be the lower bound and upper bound in radians or
            %for linear in units of min to max
            
            if function_input_type == "rotation"
                obj.input_type = 1;
                obj.input_limits = limits;
                obj.current_direction = 1;
            end
        end
        
        function update_now(obj, interval, total_interval, internal_which_link)
            %Internal_which_link also seems to have been depricated in some
            %iteration of this function, it now seems to serve no purpose
            %as well though every function call has a position for
            %internal_which_link therefore it cannot be deleted without
            %first checking every instance of update_now.
            if obj.input_type == 0
                %check which type of link this is and choose functin
                %accordingly
                if isa(obj.input_link, 'link')
                    new_input_point = obj.input_link.joint_points(obj.input_link.output_joints, :);
                elseif isa(obj.input_link, 'pivot_link')
                    new_input_point = obj.input_link.get_point();
                end
                %Gets point data
                old_pivot_point = obj.joint_points(obj.pivot_joints, :);
                old_input_point = obj.joint_points(obj.input_joints, :);
                old_output_point = obj.joint_points(obj.output_joints, :);
                
                %Obtains current pre shifted angles and lengths of vectors
                [theta_input, rho_input] = cart2pol((old_input_point(1) - old_pivot_point(1)),(old_input_point(2) - old_pivot_point(2)));
                [theta_output, rho_output] = cart2pol((old_output_point(1) - old_pivot_point(1)),(old_output_point(2) - old_pivot_point(2)));
                
                %Gets new point that will be shifted to
                new_pivot_point = obj.pivot_links.get_point();
                
                %Obtains new angle for vector from pivot to input
                [new_theta_input, new_rho_input] = cart2pol((new_input_point(1) - new_pivot_point(1)),(new_input_point(2) - new_pivot_point(2)));
                
                %Calculated change required for old vector from pivot to
                %output
                theta_change = new_theta_input - theta_input;
                new_theta_output = theta_change + theta_output;
                [output_x_change, output_y_change] = pol2cart(new_theta_output, rho_output);
                new_output = [output_x_change + new_pivot_point(1), output_y_change+ new_pivot_point(2)];
                
                %Saves new data points
                obj.old_output_point = obj.joint_points(obj.output_joints,:);
                obj.joint_points(obj.input_joints,:) = new_input_point;
                obj.joint_points(obj.output_joints,:) = new_output;
                obj.joint_points(obj.pivot_joints,:) = new_pivot_point;
                
            %If this link is an input link of rotation type
            elseif obj.input_type == 1
                %Rotate the link and update all points of it
                theta_change = interval*2*pi/total_interval;
                for i = 1:obj.number_of_joints
                    if i == obj.input_joints
                        continue
                    end
                    [old_theta, rho] = obj.get_vector(obj.input_joints,i);
                    new_theta = old_theta + theta_change;
                    [new_x, new_y] = pol2cart(new_theta, rho);
                    obj.joint_points(i,:) = obj.joint_points(obj.input_joints,:) + [new_x, new_y];
                end
            end
        end
        
        function draw_now(obj, color)
            %Draw the link
            for i = 1:obj.number_of_joints - 1
                plot([obj.joint_points(i,1),obj.joint_points(i+1,1)], [obj.joint_points(i,2),obj.joint_points(i+1,2)], color, 'LineWidth',5)
            end
        end
        
        function draw_velocity(obj,time)
            %Draw the velocity of the link in its movement
            old_point = obj.old_output_point;
            new_point = obj.joint_points(obj.output_joints,:);
            [theta, rho] = cart2pol(new_point(1) - old_point(1), new_point(2) - old_point(2));
            [x, y] = pol2cart(theta, rho/time);
            plot([new_point(1),new_point(1) + x],[new_point(2),new_point(2) + y]);
        end
                
        
        function joint_points = return_points(obj)
            %Return the joint coordinates
            joint_points = obj.joint_points;
        end
    end
end

