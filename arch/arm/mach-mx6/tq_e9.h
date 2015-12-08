#ifndef _TQ_E9_H
#define _TQ_E9_H
int mx6q_sabresd_fec_phy_init(struct phy_device *phydev);
int tq_tsc_exit_hw(void);
int tq_tsc_init_hw(void);
int tq_tsc_state_hw(void);
int mx6q_sabresd_sata_init(struct device *dev, void __iomem *mmio);
void mx6q_sabresd_sata_exit(struct device *dev);
#endif
