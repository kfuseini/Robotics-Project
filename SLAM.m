classdef SLAM
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot;
    end
    
    methods
        function obj = SLAM(fname, range)
            obj.robot = Robot(fname, range);
        end
        
        function run(obj)
            bestPos = [-1 -1];
            prevPos = [-1 -1];
            prevBox = [-1 -1];
            obj.robot.map.showEnv();
            while(bestPos ~= transpose(obj.robot.map.goal))
                obj.robot.scan();
                pause(0.1);
                path = SSS(obj.robot.map).mainLoop();
                if(isempty(path))
                    disp('NO PATH AVAILABLE');
                    return;
                end
                prevPos = bestPos;
                for pos = path
                    point = [pos{1}(1) pos{1}(2)];
                    sep = Geom2d().sep(point, obj.robot.currPos);
                    if(sep < obj.robot.laser.range - obj.robot.map.radius & point ~= prevBox)
                        bestPos = point;
                    end
                end
                if(prevPos == bestPos)
                    prevBox = bestPos;
                    bestPos = obj.closest(prevPos, [path{2}(1) path{2}(2)]);
                end
                disp(bestPos);
                obj.robot.map.showEnv();
                drawnow;
                viscircles(obj.robot.currPos, obj.robot.laser.range, 'LineStyle','--');
                obj.robot.currPos = bestPos;
            end
            obj.robot.map.start = bestPos';
            obj.robot.map.showEnv();
            drawnow;
            viscircles(obj.robot.currPos, obj.robot.laser.range, 'LineStyle','--');
            
            
        end
        function point = closest(obj, currPos, nextPos)
            inters = Line(currPos, nextPos).intersecs(currPos, obj.robot.laser.range - obj.robot.map.radius);
            point = inters(2);
            point = point{1};
            if(point == currPos)
                point = inters(1);
                point = point{1};
            end
        end
    end
    
    methods(Static)
        function a = test(fname, range)
            if(nargin == 1)
                range = 1;
            elseif (nargin < 1)
                fname = 'env1.txt';
                range = 1;
            end
            a = SLAM(fname, range);
            a.run();
        end
    end
    
end

