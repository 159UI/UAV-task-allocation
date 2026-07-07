function uav_main_gui
%UAV_MAIN_GUI 左侧面板精确定位无遮挡
    data.scenario=[]; data.sa_result=[]; data.cs_result=[]; data.sa_sim=[]; data.cs_sim=[];
    data.current_algo='SA'; data.current_tab='path';

    screen=get(0,'ScreenSize');
    fig=figure('Name','UAV任务分配仿真 — SA vs CS','NumberTitle','off',...
        'Position',[(screen(3)-1250)/2,(screen(4)-780)/2,1250,780],...
        'Resize','on','Color',[0.94,0.94,0.96],'MenuBar','none');

    pl=uipanel(fig,'Position',[0.005,0.01,0.27,0.98],...
        'Title','控制面板','FontSize',12,'FontWeight','bold','BackgroundColor',[0.94,0.94,0.96]);
    pw=0.94; px=0.03; bg=[0.97,0.97,0.98]; gap=0.012;

    % ===== 从底向上排布 =====
    bot_m=0.005;
    h6=0.20; y6=bot_m;
    h5=0.14; y5=y6+h6+gap;
    h4=0.18; y4=y5+h5+gap;
    h3=0.11; y3=y4+h4+gap;
    h2=0.16; y2=y3+h3+gap;
    h1=0.08; y1=y2+h2+gap;

    % ① 算法选择
    p1=uipanel(pl,'Position',[px,y1,pw,h1],'Title','① 算法选择','FontSize',10,'FontWeight','bold','BackgroundColor',bg);
    uicontrol(p1,'Style','text','String','选择算法:','Position',[10,18,65,22],'HorizontalAlignment','left','FontSize',10);
    h_algo=uicontrol(p1,'Style','popupmenu','String',{'SA（模拟退火）','CS（布谷鸟搜索）'},...
        'Position',[75,16,190,28],'FontSize',10,'FontWeight','bold','BackgroundColor','white','Value',1,...
        'Callback',@(~,~)algo_sel());

    % ② 场景参数
    p2=uipanel(pl,'Position',[px,y2,pw,h2],'Title','② 场景参数（静态）','FontSize',10,'FontWeight','bold','BackgroundColor',bg);
    uicontrol(p2,'Style','text','String','无人机 N:','Position',[10,80,60,20],'HorizontalAlignment','left','FontSize',9);
    h_N=uicontrol(p2,'Style','edit','String','5','Position',[70,78,50,24],'BackgroundColor','white');
    uicontrol(p2,'Style','text','String','目标数 M:','Position',[135,80,60,20],'HorizontalAlignment','left','FontSize',9);
    h_M=uicontrol(p2,'Style','edit','String','12','Position',[195,78,50,24],'BackgroundColor','white');
    uicontrol(p2,'Style','text','String','障碍数 K:','Position',[10,42,60,20],'HorizontalAlignment','left','FontSize',9);
    h_K=uicontrol(p2,'Style','edit','String','6','Position',[70,40,50,24],'BackgroundColor','white');
    uicontrol(p2,'Style','text','String','最大航程:','Position',[135,42,65,20],'HorizontalAlignment','left','FontSize',9);
    h_Lmax=uicontrol(p2,'Style','edit','String','800','Position',[200,40,55,24],'BackgroundColor','white');

    % ③ 无人机速度
    p3=uipanel(pl,'Position',[px,y3,pw,h3],'Title','③ 无人机速度','FontSize',10,'FontWeight','bold','BackgroundColor',bg);
    uicontrol(p3,'Style','text','String','速度（逗号分隔，长度=N）:','Position',[10,50,170,18],'HorizontalAlignment','left','FontSize',9);
    h_spd=uicontrol(p3,'Style','edit','String','8,10,6,8,7','Position',[180,48,110,24],'BackgroundColor','white');
    uicontrol(p3,'Style','text','String','例: 8,10,6,8,7 表示5架UAV各自速度',...
        'Position',[10,16,280,16],'HorizontalAlignment','left','FontSize',8,'ForegroundColor',[0.5,0.5,0.5]);

    % ④ SA 参数
    sa1=70; sa2=32;
    p_sa=uipanel(pl,'Position',[px,y4,pw,h4],'Title','④ SA（模拟退火）参数','FontSize',10,'FontWeight','bold',...
        'BackgroundColor',[1,0.97,0.97]);
    uicontrol(p_sa,'Style','text','String','初始温度 T0:','Position',[8,sa1,72,18],'HorizontalAlignment','left','FontSize',9);
    h_T0=uicontrol(p_sa,'Style','edit','String','100','Position',[80,sa1-2,55,24],'BackgroundColor','white');
    uicontrol(p_sa,'Style','text','String','降温率 α:','Position',[150,sa1,55,18],'HorizontalAlignment','left','FontSize',9);
    h_alp=uicontrol(p_sa,'Style','edit','String','0.95','Position',[205,sa1-2,55,24],'BackgroundColor','white');
    uicontrol(p_sa,'Style','text','String','终止温度 Tend:','Position',[8,sa2,85,18],'HorizontalAlignment','left','FontSize',9);
    h_Tend=uicontrol(p_sa,'Style','edit','String','1','Position',[93,sa2-2,55,24],'BackgroundColor','white');
    uicontrol(p_sa,'Style','text','String','最大迭代:','Position',[160,sa2,55,18],'HorizontalAlignment','left','FontSize',9);
    h_sai=uicontrol(p_sa,'Style','edit','String','500','Position',[215,sa2-2,55,24],'BackgroundColor','white');

    % ④ CS 参数
    p_cs=uipanel(pl,'Position',[px,y4,pw,h4],'Title','④ CS（布谷鸟搜索）参数','FontSize',10,'FontWeight','bold',...
        'BackgroundColor',[0.97,0.97,1],'Visible','off');
    uicontrol(p_cs,'Style','text','String','种群 Npop:','Position',[8,sa1,72,18],'HorizontalAlignment','left','FontSize',9);
    h_Npop=uicontrol(p_cs,'Style','edit','String','25','Position',[80,sa1-2,55,24],'BackgroundColor','white');
    uicontrol(p_cs,'Style','text','String','发现概率 Pa:','Position',[150,sa1,65,18],'HorizontalAlignment','left','FontSize',9);
    h_Pa=uicontrol(p_cs,'Style','edit','String','0.25','Position',[215,sa1-2,55,24],'BackgroundColor','white');
    uicontrol(p_cs,'Style','text','String','步长 α_step:','Position',[8,sa2,80,18],'HorizontalAlignment','left','FontSize',9);
    h_css=uicontrol(p_cs,'Style','edit','String','0.01','Position',[88,sa2-2,55,24],'BackgroundColor','white');
    uicontrol(p_cs,'Style','text','String','最大迭代:','Position',[158,sa2,55,18],'HorizontalAlignment','left','FontSize',9);
    h_csi=uicontrol(p_cs,'Style','edit','String','500','Position',[213,sa2-2,55,24],'BackgroundColor','white');

    % ⑤ 权重
    p5=uipanel(pl,'Position',[px,y5,pw,h5],'Title','⑤ 权重','FontSize',10,'FontWeight','bold','BackgroundColor',bg);
    uicontrol(p5,'Style','text','String','航程权重 ω:','Position',[8,68,72,20],'HorizontalAlignment','left','FontSize',9);
    h_om=uicontrol(p5,'Style','edit','String','0.3','Position',[80,66,55,24],'BackgroundColor','white');
    uicontrol(p5,'Style','text','String','惩罚 P:','Position',[8,28,50,20],'HorizontalAlignment','left','FontSize',9);
    h_Pv=uicontrol(p5,'Style','edit','String','50','Position',[58,26,50,24],'BackgroundColor','white');
    uicontrol(p5,'Style','text','String','安全距离 d_safe:','Position',[125,28,90,20],'HorizontalAlignment','left','FontSize',9);
    h_ds=uicontrol(p5,'Style','edit','String','2','Position',[215,26,45,24],'BackgroundColor','white');

    % ⑥ 操作
    p6=uipanel(pl,'Position',[px,y6,pw,h6],'Title','⑥ 操作','FontSize',10,'FontWeight','bold','BackgroundColor',bg);
    bw=130; bh=32;
    uicontrol(p6,'Style','pushbutton','String','🔄 生成场景','Position',[10,110,bw,bh],'FontWeight','bold','FontSize',10,...
        'Callback',@(~,~)gen_cb());
    uicontrol(p6,'Style','pushbutton','String','▶ 运行当前算法','Position',[10+bw+8,110,bw+15,bh],...
        'FontWeight','bold','FontSize',10,'BackgroundColor',[0.7,0.9,0.7],...
        'Callback',@(~,~)run_cb());
    uicontrol(p6,'Style','pushbutton','String','📊 对比分析','Position',[10,62,bw,bh-4],'FontWeight','bold','FontSize',9,...
        'Callback',@(~,~)comp_cb());
    uicontrol(p6,'Style','pushbutton','String','📈 统计运行','Position',[10+bw+8,62,bw+15,bh-4],...
        'FontWeight','bold','FontSize',9,'BackgroundColor',[1.0,0.9,0.7],...
        'Callback',@(~,~)stats_cb());
    uicontrol(p6,'Style','pushbutton','String','🔁 重置','Position',[10,15,65,28],'FontSize',9,...
        'Callback',@(~,~)reset_cb());
    uicontrol(p6,'Style','text','String','种子:','Position',[85,18,35,16],'HorizontalAlignment','left','FontSize',8);
    h_sd=uicontrol(p6,'Style','edit','String','','Position',[115,15,50,22],'BackgroundColor','white','FontSize',8);
    uicontrol(p6,'Style','text','String','运行次数:','Position',[178,18,55,16],'HorizontalAlignment','left','FontSize',8);
    h_nr=uicontrol(p6,'Style','edit','String','10','Position',[230,15,35,22],'BackgroundColor','white','FontSize',8);

    h_st=uicontrol(pl,'Style','text','String','就绪。设参数 → 生成场景 → 运行算法。',...
        'Position',[10,3,320,22],'HorizontalAlignment','left','FontSize',9,'ForegroundColor',[0.3,0.3,0.3]);

    % ========== 右侧结果 ==========
    pr=uipanel(fig,'Position',[0.285,0.01,0.71,0.98],'Title','结果展示','FontSize',12,'FontWeight','bold','BackgroundColor',[1,1,1]);

    h_t1=uicontrol(pr,'Style','pushbutton','String','🗺 路径图','Position',[20,648,100,28],...
        'FontSize',9,'FontWeight','bold','BackgroundColor',[0.8,0.9,1.0],...
        'Callback',@(~,~)sw('path'));
    h_t2=uicontrol(pr,'Style','pushbutton','String','📈 收敛曲线','Position',[125,648,110,28],...
        'FontSize',9,'Callback',@(~,~)sw('conv'));
    h_t3=uicontrol(pr,'Style','pushbutton','String','📋 分配表','Position',[240,648,100,28],...
        'FontSize',9,'Callback',@(~,~)sw('table'));

    ax=axes(pr,'Position',[0.04,0.06,0.94,0.86],'Box','on');
    h_info=uicontrol(pr,'Style','text','String','就绪。','Position',[20,4,880,20],...
        'HorizontalAlignment','left','FontSize',10,'ForegroundColor',[0.2,0.2,0.2]);

    % ========== 回调 ==========
    function algo_sel()
        if get(h_algo,'Value')==1
            data.current_algo='SA'; set(p_sa,'Visible','on'); set(p_cs,'Visible','off');
        else
            data.current_algo='CS'; set(p_sa,'Visible','off'); set(p_cs,'Visible','on');
        end
        set(h_info,'String',['已切换至: ' data.current_algo]);
    end

    function sw(tab)
        data.current_tab=tab;
        set(h_t1,'BackgroundColor',[0.94,0.94,0.96]);
        set(h_t2,'BackgroundColor',[0.94,0.94,0.96]);
        set(h_t3,'BackgroundColor',[0.94,0.94,0.96]);
        switch tab
            case 'path', set(h_t1,'BackgroundColor',[0.8,0.9,1.0]);
            case 'conv', set(h_t2,'BackgroundColor',[0.8,0.9,1.0]);
            case 'table', set(h_t3,'BackgroundColor',[0.8,0.9,1.0]);
        end
        ref();
    end

    function ref()
        if isempty(data.scenario)
            cla(ax); text(ax,0.5,0.5,'请先生成场景','HorizontalAlignment','center','FontSize',16); axis(ax,'off');
            delete(findobj(pr,'Type','uitable')); return;
        end
        delete(findobj(pr,'Type','uitable'));
        for a=findall(pr,'Type','axes')'; if a~=ax, delete(a); end; end
        switch data.current_tab
            case 'path', vpath();
            case 'conv', vconv();
            case 'table', vtable();
        end
        drawnow('limitrate');
    end

    function vpath()
        [r,an]=getr();
        if ~isempty(r)
            tg=sum(cellfun(@length,r.best_sequence));
            plot_path(ax,data.scenario,r.best_sequence,...
                sprintf('%s  J=%.2f  访问%d/%d目标',an,r.best_fitness,tg,data.scenario.M));
            set(h_info,'String',sprintf('%s: J=%.2f 距离=%.1f 耗时=%.3fs 访问%d/%d目标',...
                an,r.best_fitness,sum(r.best_distances),r.runtime,tg,data.scenario.M));
        else
            plot_path(ax,data.scenario,cell(data.scenario.N,1),'场景（未运行算法）');
            set(h_info,'String',sprintf('场景: N=%d, M=%d, K=%d',data.scenario.N,data.scenario.M,data.scenario.K));
        end
    end

    function vconv()
        sh=[]; ch=[]; st=0; ct=0;
        if ~isempty(data.sa_result), sh=data.sa_result.fitness_history; st=data.sa_result.runtime; end
        if ~isempty(data.cs_result), ch=data.cs_result.fitness_history; ct=data.cs_result.runtime; end
        plot_convergence(ax,sh,ch,st,ct);
        i='收敛曲线: ';
        if ~isempty(data.sa_result), i=[i sprintf('SA J=%.2f ',data.sa_result.best_fitness)]; end
        if ~isempty(data.cs_result), i=[i sprintf('CS J=%.2f',data.cs_result.best_fitness)]; end
        set(h_info,'String',i);
    end

    function vtable()
        cla(ax); axis(ax,'off');
        x0=0.02; w=0.46;
        if ~isempty(data.sa_result)
            a1=axes(pr,'Position',[x0,0.55,w,0.35]);
            plot_allocation_table(a1,data.scenario,data.sa_result.best_sequence,data.sa_result.best_distances,'SA 分配');
        end
        if ~isempty(data.cs_result)
            a2=axes(pr,'Position',[x0+w+0.03,0.55,w,0.35]);
            plot_allocation_table(a2,data.scenario,data.cs_result.best_sequence,data.cs_result.best_distances,'CS 分配');
        end
        t='统计对比:\n';
        if ~isempty(data.sa_result), t=[t sprintf('SA: J=%.2f 距离=%.1f 耗时=%.3fs\n',...
            data.sa_result.best_fitness,sum(data.sa_result.best_distances),data.sa_result.runtime)]; end
        if ~isempty(data.cs_result), t=[t sprintf('CS: J=%.2f 距离=%.1f 耗时=%.3fs\n',...
            data.cs_result.best_fitness,sum(data.cs_result.best_distances),data.cs_result.runtime)]; end
        if ~isempty(data.sa_result)&&~isempty(data.cs_result)
            d=data.sa_result.best_fitness-data.cs_result.best_fitness;
            if abs(d)<0.5, t=[t '结论: 两算法性能接近'];
            elseif d>0, t=[t sprintf('结论: SA优于CS (ΔJ=%.2f)',d)];
            else t=[t sprintf('结论: CS优于SA (ΔJ=%.2f)',-d)]; end
        end
        text(ax,0.02,0.45,strrep(t,'\n',newline),'VerticalAlignment','top','FontName','Consolas','FontSize',9);
        set(h_info,'String','分配方案与统计对比');
    end

    function [r,an]=getr()
        if strcmp(data.current_algo,'SA')&&~isempty(data.sa_result), r=data.sa_result; an='SA'; return; end
        if strcmp(data.current_algo,'CS')&&~isempty(data.cs_result), r=data.cs_result; an='CS'; return; end
        if ~isempty(data.sa_result), r=data.sa_result; an='SA'; return; end
        if ~isempty(data.cs_result), r=data.cs_result; an='CS'; return; end
        r=[]; an='';
    end

    function s=getsp()
        s=struct();
        s.N=round(str2double(get(h_N,'String'))); s.M=round(str2double(get(h_M,'String')));
        s.K=round(str2double(get(h_K,'String'))); s.Lmax=str2double(get(h_Lmax,'String'));
        s.om=str2double(get(h_om,'String')); s.Pv=str2double(get(h_Pv,'String')); s.ds=str2double(get(h_ds,'String'));
        s.sv=str2double(strsplit(strtrim(get(h_spd,'String')),','));
        s.nr=round(str2double(get(h_nr,'String')));
        ss=strtrim(get(h_sd,'String'));
        if ~isempty(ss), s.sd=str2double(ss); else s.sd=[]; end
    end

    function p=sap()
        p.T0=str2double(get(h_T0,'String')); p.alpha=str2double(get(h_alp,'String'));
        p.T_end=str2double(get(h_Tend,'String')); p.max_iter=round(str2double(get(h_sai,'String')));
    end
    function p=csp()
        p.N_pop=round(str2double(get(h_Npop,'String'))); p.Pa=str2double(get(h_Pa,'String'));
        p.alpha_step=str2double(get(h_css,'String')); p.max_iter=round(str2double(get(h_csi,'String')));
    end

    function ok=chk(s)
        ok=false; e={};
        if any(isnan([s.N,s.M,s.K,s.Lmax,s.om,s.Pv])),e{end+1}='无效数值'; end
        if s.N<1,e{end+1}='N≥1'; end; if s.M<1,e{end+1}='M≥1'; end
        if s.nr<1||s.nr>100,e{end+1}='运行次数1~100'; end
        if isempty(e),ok=true; else set(h_st,'String',['错误: ' strjoin(e,';')],'ForegroundColor','red'); end
    end

    function gen_cb()
        s=getsp(); if ~chk(s), return; end
        if length(s.sv)<s.N, s.sv=s.sv(1)*ones(1,s.N); end
        try
            data.scenario=generate_scenario(s.N,s.M,s.K,'L_max',s.Lmax,...
                'uav_speed',s.sv(1:s.N),...
                'omega',s.om,'P',s.Pv,'d_safe',s.ds,'seed',s.sd);
            data.sa_result=[]; data.cs_result=[]; data.sa_sim=[]; data.cs_sim=[]; sw('path');
            set(h_st,'String',sprintf('场景: N=%d, M=%d, K=%d',s.N,s.M,s.K),'ForegroundColor',[0,0.5,0]);
        catch ME, set(h_st,'String',['失败: ' ME.message],'ForegroundColor','red'); end
    end

    function run_cb()
        if isempty(data.scenario), set(h_st,'String','请先生成场景!','ForegroundColor','red'); return; end
        s=getsp(); data.scenario.omega=s.om; data.scenario.P=s.Pv; data.scenario.d_safe=s.ds;
        try
            if strcmp(data.current_algo,'SA')
                p=sap(); set(h_st,'String','运行SA中...','ForegroundColor',[0,0,0.6]); drawnow;
                data.sa_result=simulated_annealing(data.scenario,p);
                data.sa_sim=simulate_dynamic(data.scenario,data.sa_result.best_sequence,data.sa_result.best_distances);
                tg=sum(cellfun(@length,data.sa_result.best_sequence));
                set(h_st,'String',sprintf('SA: J=%.2f %.3fs 访问%d/%d目标',...
                    data.sa_result.best_fitness,data.sa_result.runtime,tg,data.scenario.M),'ForegroundColor',[0,0,0.6]);
            else
                p=csp(); set(h_st,'String','运行CS中...','ForegroundColor',[0.6,0,0]); drawnow;
                data.cs_result=cuckoo_search(data.scenario,p);
                data.cs_sim=simulate_dynamic(data.scenario,data.cs_result.best_sequence,data.cs_result.best_distances);
                tg=sum(cellfun(@length,data.cs_result.best_sequence));
                set(h_st,'String',sprintf('CS: J=%.2f %.3fs 访问%d/%d目标',...
                    data.cs_result.best_fitness,data.cs_result.runtime,tg,data.scenario.M),'ForegroundColor',[0.6,0,0]);
            end
            sw('path');
        catch ME, set(h_st,'String',['运行失败: ' ME.message],'ForegroundColor','red'); end
    end

    function comp_cb()
        if isempty(data.sa_result)&&isempty(data.cs_result)
            set(h_st,'String','请先运行算法!','ForegroundColor','red'); return; end
        sw('conv'); set(h_st,'String','收敛曲线对比','ForegroundColor',[0.5,0,0.5]);
    end

    function stats_cb()
        if isempty(data.scenario), set(h_st,'String','请先生成场景!','ForegroundColor','red'); return; end
        s=getsp(); sap_=sap(); csp_=csp();
        nr=max(1,min(100,s.nr));
        sa_f=zeros(nr,1); cs_f=zeros(nr,1);
        for r=1:nr
            sd=s.sd; if ~isempty(sd), sd=sd+r; end
            sc=generate_scenario(s.N,s.M,s.K,'L_max',s.Lmax,...
                'uav_speed',s.sv(1:s.N),...
                'omega',s.om,'P',s.Pv,'d_safe',s.ds,'seed',sd);
            sa_f(r)=simulated_annealing(sc,sap_).best_fitness;
            cs_f(r)=cuckoo_search(sc,csp_).best_fitness;
            set(h_st,'String',sprintf('统计 %d/%d...',r,nr),'ForegroundColor',[0.5,0.3,0]); drawnow('limitrate');
        end
        cla(ax);
        bar(ax,[mean(sa_f),mean(cs_f);std(sa_f),std(cs_f);max(sa_f),max(cs_f)]);
        set(ax,'XTickLabel',{'平均值','标准差','最优值'});
        colormap(ax,[0.3,0.5,0.8;0.8,0.3,0.3]);
        legend(ax,{'SA','CS'},'Location','northwest');
        title(ax,sprintf('统计对比（%d次）',nr),'FontWeight','bold'); grid(ax,'on');
        t=sprintf('=== %d次统计 ===\nSA: 均值=%.2f 标准差=%.2f 最优=%.2f\nCS: 均值=%.2f 标准差=%.2f 最优=%.2f',...
            nr,mean(sa_f),std(sa_f),max(sa_f),mean(cs_f),std(cs_f),max(cs_f));
        text(ax,0.02,-0.15,strrep(t,'\n',newline),'VerticalAlignment','top','FontName','Consolas','FontSize',9,'Units','normalized');
        set(h_st,'String',sprintf('统计: SA=%.2f CS=%.2f (%d次)',mean(sa_f),mean(cs_f),nr),'ForegroundColor',[0.5,0.3,0]);
    end

    function reset_cb()
        data.scenario=[]; data.sa_result=[]; data.cs_result=[]; data.sa_sim=[]; data.cs_sim=[];
        cla(ax); axis(ax,'off'); delete(findobj(pr,'Type','uitable'));
        set(h_st,'String','已重置。','ForegroundColor',[0.3,0.3,0.3]); set(h_info,'String','已重置');
    end

    set(h_st,'String','就绪。设参数 → 生成场景 → 运行算法。');
end
