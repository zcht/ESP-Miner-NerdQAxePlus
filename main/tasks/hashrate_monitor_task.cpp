#include "global_state.h"
#include "hashrate_monitor_task.h"
#include "boards/board.h"
#include "esp_log.h"
#include "mining.h"
#include "utils.h"
#include <string.h>

static const char *HR_TAG = "hashrate_monitor";
// Keep enough gap so every ASIC in the chain can answer each register read
// before the next command is sent (important on slower effective UART rates).
static constexpr uint32_t HR_REG_READ_GAP_MS = 12;
HashrateMonitor::HashrateMonitor()
{}

bool HashrateMonitor::start(Board *board, Asic *asic)
{
    m_board = board;
    m_asic = asic;
    m_period_ms = HR_INTERVAL;

    if (!m_board || !m_asic) {
        ESP_LOGE(HR_TAG, "start(): missing dependencies (board=%p, asic=%p)", (void *) m_board, (void *) m_asic);
        return false;
    }

    m_asicCount = board->getAsicCount();
    m_domainCount = 0;
    if (m_asic) {
        m_domainCount = m_asic->getHashDomainCount();
        if (m_domainCount < 0) m_domainCount = 0;
        if (m_domainCount > 4) m_domainCount = 4;
    }

    const char *asicModel = m_board->getAsicModel();
    const bool isBm136xOr1370 = asicModel &&
        (strcmp(asicModel, "BM1366") == 0 ||
         strcmp(asicModel, "BM1368") == 0 ||
         strcmp(asicModel, "BM1370") == 0);

    // BM1366/BM1368/BM1370 expose total hash counters on 0x8C.
    // BM1397-class boards keep the legacy 0x90 path.
    m_totalCounterReg = isBm136xOr1370 ? REG_NONCE_TOTAL_CNT_DOMAIN : REG_NONCE_TOTAL_CNT_DEFAULT;

    m_chipHashrate = new float[m_asicCount]();

    if (m_domainCount > 0 && m_asicCount > 0) {
        const size_t totalDomains = static_cast<size_t>(m_asicCount) * static_cast<size_t>(m_domainCount);
        m_domainHashrate = new float[totalDomains]();
        m_prevDomainResponse = new int64_t[totalDomains]();
        m_prevDomainCounter = new uint32_t[totalDomains]();
    }

    m_prevResponse = new int64_t[m_asicCount]();
    m_prevCounter = new uint32_t[m_asicCount]();


    xTaskCreatePSRAM(&HashrateMonitor::taskWrapper, "hr_monitor", 4096, (void *) this, 10, NULL);
    ESP_LOGI(HR_TAG, "started (period=%lums)", m_period_ms);
    return true;
}

void HashrateMonitor::setChipHashrate(int nr, float temp) {
    if (nr < 0 || nr >= m_asicCount) {
        return;
    }
    m_chipHashrate[nr] = temp;
}

void HashrateMonitor::setDomainHashrate(int asicIdx, int domainIdx, float ghs) {
    if (!m_domainHashrate || m_domainCount <= 0) return;
    if (asicIdx < 0 || asicIdx >= m_asicCount) return;
    if (domainIdx < 0 || domainIdx >= m_domainCount) return;
    const size_t idx = static_cast<size_t>(asicIdx) * static_cast<size_t>(m_domainCount) + static_cast<size_t>(domainIdx);
    m_domainHashrate[idx] = ghs;
}

float HashrateMonitor::getDomainHashrateInternal(int asicIdx, int domainIdx) {
    if (!m_domainHashrate || m_domainCount <= 0) return 0.0f;
    if (asicIdx < 0 || asicIdx >= m_asicCount) return 0.0f;
    if (domainIdx < 0 || domainIdx >= m_domainCount) return 0.0f;
    const size_t idx = static_cast<size_t>(asicIdx) * static_cast<size_t>(m_domainCount) + static_cast<size_t>(domainIdx);
    return m_domainHashrate[idx];
}

float HashrateMonitor::getChipHashrate(int nr) {
    if (nr < 0 || nr >= m_asicCount) {
        return 0.0f;
    }
    return m_chipHashrate[nr];
}

float HashrateMonitor::getTotalChipHashrate() {
    float total = 0.0f;
    for (int i=0;i < m_asicCount; i++) {
        total += m_chipHashrate[i];
    }
    return total;
}

void HashrateMonitor::taskWrapper(void *pv)
{
    auto *self = static_cast<HashrateMonitor *>(pv);
    self->taskLoop();
}

void HashrateMonitor::publishTotalIfComplete()
{
    size_t offset = 0;

    Board* board = SYSTEM_MODULE.getBoard();

    // Iterate through each ASIC and append its count to the log message
    for (int i = 0; i < board->getAsicCount(); i++) {
        offset += snprintf(m_logBuffer + offset, sizeof(m_logBuffer) - offset, "%.2fGH/s / ", getChipHashrate(i));
    }
    if (offset >= 2) {
        m_logBuffer[offset - 2] = 0; // remove trailing slash
    }

    // apply slight 3 tap median filter to remove weird outliers
    m_hashrate = m_median.update(getTotalChipHashrate());

    ESP_LOGI(HR_TAG, "chip hashrates: %s (total: %.3fGH/s)", m_logBuffer, m_hashrate);
}

void HashrateMonitor::taskLoop()
{
    // Small startup delay
    vTaskDelay(pdMS_TO_TICKS(4000));

    // Important for BM1366/BM1368/BM1370:
    // treat hashrate registers as read-only and never write/reset them.
    // We seed the baseline from first read responses in onRegisterReply().

    TickType_t lastWake = xTaskGetTickCount();
    while (1) {
        if (POWER_MANAGEMENT_MODULE.isShutdown()) {
            ESP_LOGW(HR_TAG, "suspended");
            vTaskSuspend(NULL);
        }

        if (!m_board || !m_asic) {
            vTaskDelay(pdMS_TO_TICKS(m_period_ms));
            continue;
        }

        // read the counters
        m_asic->readCounter(m_totalCounterReg);
        vTaskDelay(pdMS_TO_TICKS(HR_REG_READ_GAP_MS));
        if (m_domainCount > 0) {
            m_asic->readCounter(REG_NONCE_DOMAIN0_CNT);
            vTaskDelay(pdMS_TO_TICKS(HR_REG_READ_GAP_MS));
            if (m_domainCount > 1) m_asic->readCounter(REG_NONCE_DOMAIN1_CNT);
            if (m_domainCount > 1) vTaskDelay(pdMS_TO_TICKS(HR_REG_READ_GAP_MS));
            if (m_domainCount > 2) m_asic->readCounter(REG_NONCE_DOMAIN2_CNT);
            if (m_domainCount > 2) vTaskDelay(pdMS_TO_TICKS(HR_REG_READ_GAP_MS));
            if (m_domainCount > 3) m_asic->readCounter(REG_NONCE_DOMAIN3_CNT);
            if (m_domainCount > 3) vTaskDelay(pdMS_TO_TICKS(HR_REG_READ_GAP_MS));
        }

        // responses normally take 20-30ms, so this is safe
        vTaskDelay(pdMS_TO_TICKS(500));

        publishTotalIfComplete();

        // apply a slight smoothing
        if (!m_smoothedHashrate) {
            m_smoothedHashrate = m_hashrate;
        }

        m_smoothedHashrate = 0.5f * m_smoothedHashrate + 0.5f * m_hashrate;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(m_period_ms));
    }
}

void HashrateMonitor::onRegisterReply(uint8_t reg, uint8_t asic_idx, uint32_t counterNow)
{
    if (asic_idx >= m_asicCount) {
        ESP_LOGE(HR_TAG, "respnse for invalid asic %d", (int) asic_idx);
        return;
    }

    int domainIdx = -1;
    if (reg == REG_NONCE_DOMAIN0_CNT) domainIdx = 0;
    else if (reg == REG_NONCE_DOMAIN1_CNT) domainIdx = 1;
    else if (reg == REG_NONCE_DOMAIN2_CNT) domainIdx = 2;
    else if (reg == REG_NONCE_DOMAIN3_CNT) domainIdx = 3;

    if (domainIdx >= 0 && domainIdx < m_domainCount) {
        const size_t idx = static_cast<size_t>(asic_idx) * static_cast<size_t>(m_domainCount) + static_cast<size_t>(domainIdx);
        int64_t now = esp_timer_get_time();

        if (!m_prevDomainResponse || !m_prevDomainCounter || !m_domainHashrate) {
            return;
        }

        if (!m_prevDomainResponse[idx]) {
            m_prevDomainResponse[idx] = now;
            m_prevDomainCounter[idx] = counterNow;
            return;
        }

        int64_t timeDelta = now - m_prevDomainResponse[idx];
        uint32_t counterDelta = counterNow - m_prevDomainCounter[idx];

        double chip_ghs = (double) counterDelta * (double) 0x100000000uLL / (double) timeDelta / 1000.0;
        setDomainHashrate((int) asic_idx, domainIdx, (float) chip_ghs);

        m_prevDomainCounter[idx] = counterNow;
        m_prevDomainResponse[idx] = now;
        return;
    }

    if (reg != m_totalCounterReg) {
        return;
    }

    int64_t now = esp_timer_get_time();

    // first response
    if (!m_prevResponse[asic_idx]) {
        m_prevResponse[asic_idx] = now;
        m_prevCounter[asic_idx] = counterNow;
        return;
    }

    int64_t timeDelta = now - m_prevResponse[asic_idx];
    uint32_t counterDelta = counterNow - m_prevCounter[asic_idx];

    double chip_ghs = (double) counterDelta * (double) 0x100000000uLL / (double) timeDelta / 1000.0;
//    ESP_LOGE("XXX", "m_prevResponse[%d]=%lld now=%lld m_prevCounter[%d]=%lu counterNow=%lu timeDelta=%llu counterDelta=%lu chip_ghs=%.3f",
//        (int) asic_idx, m_prevResponse[asic_idx], now, (int) asic_idx, m_prevCounter[asic_idx], counterNow, timeDelta, counterDelta, chip_ghs);

    setChipHashrate(asic_idx, chip_ghs);

    m_prevCounter[asic_idx] = counterNow;
    m_prevResponse[asic_idx] = now;
}

float HashrateMonitor::getDomainHashrate(int asicIdx, int domainIdx) {
    return getDomainHashrateInternal(asicIdx, domainIdx);
}
