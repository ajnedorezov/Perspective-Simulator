classdef Pixel < handle
    properties
        
        pixelNumber = 0;
        r = 0;                                                              % row
        c = 0;                                                              % column
        feat  = [0 0 0];                                                    % feature vector
        pixSize = [0 0];                                                    % height & width of pixel
        validPixel = false;
        
        regularization = 40;
        
        rHist = []
        cHist = [];
        featHist = [];
        rDeltaHist = [];
        cDeltaHist = [];
        rDeltaHistEnd = 0;
        cDeltaHistEnd = 0;
        featDeltaHist = [];
        rAvg = 0;
        cAvg = 0;
        rDeltaAvg = 0;
        cDeltaAvg = 0;
        updateCounter = 0;
        
        Image
        imr
        imc
        imsize_
    end
    
    methods
        function self = Pixel(n,r,c,feat, size)
            self.pixelNumber = n;
            self.r = r;
            self.c = c;
            self.feat = feat;
            self.pixSize = size;
            self.validPixel = true;
            
            self.UpdateHistory();
        end
        
        function dmat = calcDistanceMatrix(self, im)
            
            self.Image = im;
            
            imsize = size(im);
            S = max(self.pixSize);
            
            dmat = inf(imsize(1), imsize(2));
            if ~self.validPixel    
                return
            end
            
%             [self.imc, self.imr] = meshgrid(1:imsize(2), 1:imsize(1));
            if isempty(self.imsize_) || any(self.imsize_ ~= imsize)
                [self.imc, self.imr] = meshgrid(1:imsize(2), 1:imsize(1));
                self.imsize_ = imsize;
            end
            
            cgrid = max(round(self.c-S), 1):min(round(self.c+S), imsize(2));
            rgrid = max(round(self.r-S), 1):min(round(self.r+S), imsize(1));
            
%             df1 = im(:,:,1) - self.feat(1);
%             df2 = im(:,:,2) - self.feat(2);
%             df3 = im(:,:,3) - self.feat(3);
            df1 = im(rgrid,cgrid,1) - self.feat(1);
            df2 = im(rgrid,cgrid,2) - self.feat(2);
            df3 = im(rgrid,cgrid,3) - self.feat(3);
            dc = sqrt( df1.*df1 + df2.*df2 + df3.*df3 );
%             dc = sqrt( sum( bsxfun(@minus, im, reshape(self.feat, 1, 1, length(self.feat))).^2, 3) );
%             dc = zeros(size(df));
            
%             dy = self.imr - self.r;
%             dx = self.imc - self.c;
            dy = self.imr(rgrid,cgrid) - self.r;
            dx = self.imc(rgrid,cgrid) - self.c;
            ds = sqrt( dy.*dy + dx.*dx );
%             ds = sqrt( (dy).^2 + (dx).^2 );
%             ds = zeros(size(dp));
            
%             dmat = sqrt( (dc/self.regularization).^2 + (ds/S).^2 );
            dmat(rgrid,cgrid) = sqrt( (dc/self.regularization).^2 + (ds/S).^2 );
%             dmat = sqrt( (dc).^2 + ((ds/S).^2)*self.regularization^2 );
            
%             % Limit the values to be within a square area
%             ind = abs(dy) > S | abs(dx) > S;
%             dmat(ind) = nan;            

        end

        function UpdateImage(self, im)
            self.Image = im;
            
            imsize = size(im);
            
%             [self.imc, self.imr] = meshgrid(1:imsize(2), 1:imsize(1));
            if isempty(self.imsize_) || any(self.imsize_ ~= imsize)
                [self.imc, self.imr] = meshgrid(1:imsize(2), 1:imsize(1));
                self.imsize_ = imsize;
            end
        end
        
        function Update(self, ind)
            if ~self.validPixel
                return
            end

            self.r = max(floor(mean(self.imr(ind(:)))), 1);
            self.c = max(floor(mean(self.imc(ind(:)))), 1);
            self.feat = self.Image(self.r, self.c, 1:3);
            
%             [rs,cs] = find(ind);
            
%             self.pixSize = [self.Dmaxmin(rs) self.Dmaxmin(cs)];
            
            self.validPixel = any(ind(:));
        end
        
        function UpdateHistory(self)
            if ~self.validPixel    
                return
            end
            if ~isempty(self.rHist)
                self.rDeltaHistEnd = self.r - self.rHist(end);
                self.cDeltaHistEnd = self.c - self.cHist(end);
                self.rDeltaHist(end+1) = self.rDeltaHistEnd;
                self.cDeltaHist(end+1) = self.cDeltaHistEnd;
                self.featDeltaHist(end+1,:) = reshape(self.feat,1,3) - self.featHist(end,:);
                
%                 if self.updateCounter > 0
                    self.rDeltaAvg = ((self.updateCounter-1)*self.rDeltaAvg + self.rDeltaHist(end))/(self.updateCounter); 
                    self.cDeltaAvg = ((self.updateCounter-1)*self.cDeltaAvg + self.cDeltaHist(end))/(self.updateCounter);
%                 end
            end
            
            self.rHist(end+1) = self.r;
            self.cHist(end+1) = self.c;
            self.featHist(end+1,:) = self.feat;
            
            self.rAvg = (self.updateCounter*self.rAvg + self.r)/(self.updateCounter + 1);
            self.cAvg = (self.updateCounter*self.cAvg + self.c)/(self.updateCounter + 1);
            
            self.updateCounter = self.updateCounter + 1;
        end
    end
    
    methods(Static)
        function val = Dmaxmin(x)
            val = max(max(x)) - min(min(x));            
        end
    end
end