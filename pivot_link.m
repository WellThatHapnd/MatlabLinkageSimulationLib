classdef pivot_link < handle
    % Created by Nolan Robins
    % Version 0.5
    
    properties
        pivot_point
        connection_point = [0,0]
        radius
        first_run = 0
    end
    
    methods
        function obj = pivot_link(pivot_point, radius)
            obj.pivot_point = pivot_point;
            obj.radius = radius;
        end
        
        function new_points = update_point(obj, input_point, input_length, pos_neg)
              r_1 = input_length;
              r_2 = obj.radius;

              p = sum((obj.pivot_point.'-input_point.').^2);
              q = (input_point.'+obj.pivot_point.')/2+(r_1^2-r_2^2)/p/2*(obj.pivot_point.'-input_point.');
              s = ((r_1+r_2)^2-p)*(p-(r_2-r_1)^2);
              if s <= 0
                fprintf("Intersection Error");
              else
                  t = sqrt(s)/p/2*[0 -1;1 0]*(obj.pivot_point.'-input_point.');
                new_points_pos = q + t;
                new_points_neg = q - t;
              
              
                [theta_old, rho_old] = cart2pol((obj.connection_point(1) - obj.pivot_point(1)),(obj.connection_point(2) - obj.pivot_point(2)));
                [theta_pos, rho_pos] = cart2pol((new_points_pos(1) - obj.pivot_point(1)),(new_points_pos(2) - obj.pivot_point(2)));
                [theta_neg, rho_neg] = cart2pol((new_points_neg(1) - obj.pivot_point(1)),(new_points_neg(2) - obj.pivot_point(2)));
              
                pos_diff = abs(theta_pos - theta_old);
                neg_diff = abs(theta_neg - theta_old);
              
              
                if pos_diff <= neg_diff
                    new_points = new_points_pos;
                else
                    new_points = new_points_neg;
                end
              
                if ~obj.first_run
                    if pos_neg
                        new_points = new_points_pos;
                    else
                        new_points = new_points_neg;
                    end
                    obj.first_run = 1;
                end
                
                obj.connection_point = new_points;
              end
              
        end
        
        function new_points = get_point(obj)
            new_points = obj.connection_point;
        end
        
        function draw_now(obj)
            plot([obj.pivot_point(1),obj.connection_point(1)], [obj.pivot_point(2),obj.connection_point(2)],'LineWidth',3)
        end
        
        function draw_circles(obj, linkage_point, linkage_radius)
            plot(nsidedpoly(1000, 'Center', obj.pivot_point, 'Radius', obj.radius))
            plot(nsidedpoly(1000, 'Center', linkage_point, 'Radius', linkage_radius))
        end
    end
end

