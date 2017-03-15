function updateLandmarkList( h,observedll )
    for ii=1:size(observedll,1)
        for jj=1:size(h.landmark_list,2)
            if(observedll(ii,2)==h.landmark_list(jj).index)
                h.landmark_list(jj).fresh=0;
            end
        end
    end
end

