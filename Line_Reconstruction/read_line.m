clear;
id_txt = 'data/data8/idx.txt';
sp_txt = 'data/data8/sp.txt';
ep_txt = 'data/data8/ep.txt';

id_data = textread(id_txt);
sp_data = fopen(sp_txt);
ep_data = fopen(ep_txt);

for i=1:length(id_data)
    temp_id = id_data(i, :);
    temp_id(find(temp_id==0))=[];
    FormatString = repmat('%f',1,length(temp_id));
    temp_sp = cell2mat(textscan(sp_data, FormatString, 2));
    temp_ep = cell2mat(textscan(ep_data, FormatString, 2));
    
    lineset(i) = lineTrack(temp_id, temp_sp', temp_ep');
end
fclose(sp_data);
fclose(ep_data);
save lineTracks lineset

