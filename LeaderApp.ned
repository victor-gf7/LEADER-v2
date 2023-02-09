package org.car2x.veins.modules.application.traci;
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple LeaderApp extends DemoBaseApplLayer
{
    @class(veins::LeaderApp);
    double windowTime = default(10.0);
    string strategy = default("Rand");
    string fileName = default("NO_NAME");
}
