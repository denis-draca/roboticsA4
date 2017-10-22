classdef Fetch   
    properties
        limit0 
        limit1 
        limit2 
        limit3 
        limit4
        limit5 
        limit6
        
        model;
        links;
        modelName;
        
        steps = 30;
    end
    
    methods
        
        function  self = Fetch(baseTransform, name)
            self.modelName = name;
            hold on;
            self.limit0 = (350/2)*pi/180;
            self.limit1 = self.limit0;
            self.limit2 = self.limit0;
            self.limit3 = self.limit0;
            self.limit4 = (341/2)*pi/180;
            self.limit5 = self.limit4;
            self.limit6 = (540/2)*pi/180;
            
            l1 = Link('d',0.3,'a',0.1,'alpha',pi/2, ...
                'offset', 0);
            
            l2 = Link('d',1,'a',0,'alpha',-pi/2,...
                'offset',0);
            
            l3 = Link('d',1,'a',0,'alpha',pi/2,'offset',0, ...
                'offset', 0);
            
            l4 = Link('d',1,'a',0,'alpha',-pi/2,'offset',0);
            
            l5 = Link('d',1,'a',0,'alpha',pi/2,'offset',0,...
                'offset', 0);
            
            l6 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0);
            
            l7 = Link('d',0,'a',0,'alpha',0,'offset',0);
            
            self.links = [l1 l2 l3 l4 l5 l6 l7];
            
            self.model = SerialLink(self.links, 'name', name);
                      
            self.model.plotopt3d = { 'noname', 'nowrist', 'notiles'};

            for linkIndex = 0:self.model.n
               [ faceData, vertexData, plyData(linkIndex + 1)] = ...
               plyread(['j',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;

            end


            self.model.delay = 0;
            % Check if there is a robot with this name, otherwise plot one 
            if isempty(findobj('Tag', self.model.name))
                self.model.base = baseTransform;
                
                self.model.plot3d([0 0 0 0 0 0 0], 'notiles');
                camlight
            end
            view([0.5, 0.5 , 0.5]);


            self.model.delay = 0;
            handles = findobj('Tag', self.model.name);
            h = get(handles,'UserData'); 
            for i = 1:8
                 h.link(i).Children.FaceVertexCData = ...
                 [plyData(i).vertex.red, plyData(i).vertex.green,...
                 plyData(i).vertex.blue]/255 ;
                 h.link(i).Children.FaceColor = 'interp';
            end

            hold on;

        end
        
    end
    
end

