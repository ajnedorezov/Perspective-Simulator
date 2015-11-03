function [x,y,z] = Road(self,length,width)
    x = [0 0 length length];
    y = [0 width width 0];
    z = [0 0 0 0] + 0.01;
end