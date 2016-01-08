function [J, grad] = costFunction(x, z, imL, imR)

f=999.421;
width=741;
height=497;
ndisp=70;
doffs=32.778;

J=double(0);
grad=double(0);

[imGrX imGrY]=imgradientxy(imR);

for i=371:height
    for j=ndisp:width
        d=round(x*f/z(i,j)-doffs);
        %         if(d>=ndisp)
        %             d=ndisp-1;
        %         end
        %         if(d<0)
        %             d=0;
        %         end
        
        if(d<ndisp && d>=0)
            diff=double(imR(i,j-d)-imL(i,j));
            J=J+diff^2;
            grad=grad+2*diff*imGrX(i,j-d);
        end
    end
end

J=J/1000000;
grad=grad/1000000;

end