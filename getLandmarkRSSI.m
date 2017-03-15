function observedll=getLandmarkRSSI(h)
    observedll=[];
    for ii=1: size(h.landmark_list,2)
        if(h.landmark_list(ii).fresh==1)
            observedll=[observedll;h.landmark_list(ii).dist,h.landmark_list(ii).index];
        end
    end
end