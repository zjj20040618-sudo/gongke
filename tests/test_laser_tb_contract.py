"""Read-only source contracts for the dedicated TB6612 laser bridge.

These checks complement the real board adapter C regression. They neither
run hardware nor validate the laser module's electrical ratings or optics.
"""
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


def source(relative):
    return (ROOT / relative).read_text(encoding="utf-8-sig")


def without_comments(text):
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)


def brace_body(text, opening):
    depth = 0
    for index in range(opening, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[opening + 1:index]
    raise AssertionError("Unbalanced source block")


def function_body(text, name):
    text = without_comments(text)
    match = re.search(r"\b" + re.escape(name) + r"\s*\([^)]*\)\s*\{", text)
    if match is None:
        raise AssertionError(f"Function definition not found: {name}")
    return brace_body(text, match.end() - 1)


def define_values(text):
    return dict(re.findall(r"^\s*#define\s+(\w+)\s+(\w+)", text, flags=re.M))


class LaserTBSourceContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.header = source("Inc/main.h")
        cls.stub = source("tests/laser_stubs/main.h")
        cls.board = source("App/board_pins.c")
        cls.gpio = source("Src/gpio.c")
        cls.timer = source("Src/tim.c")
        cls.main = source("Src/main.c")
        cls.irq = source("Src/stm32f4xx_it.c")
        cls.rtos = source("Src/freertos.c")
        cls.ioc_lines = [
            line.split("=", 1) for line in source("jiejie.ioc").splitlines()
            if line and not line.startswith("#") and "=" in line
        ]
        cls.ioc = dict(cls.ioc_lines)

    def test_actual_four_pin_mappings_match_test_stubs(self):
        real = define_values(self.header)
        stub = define_values(self.stub)
        expected = {
            "LASER_PWMA": ("GPIO_PIN_9", "GPIOB"),
            "LASER_AIN1": ("GPIO_PIN_0", "GPIOB"),
            "LASER_AIN2": ("GPIO_PIN_1", "GPIOB"),
            "LASER_STBY": ("GPIO_PIN_12", "GPIOC"),
        }
        for label, (pin, port) in expected.items():
            with self.subTest(label=label):
                self.assertEqual(real[label + "_Pin"], pin)
                self.assertEqual(real[label + "_GPIO_Port"], port)
                self.assertEqual(stub[label + "_Pin"], pin)
                self.assertEqual(stub[label + "_GPIO_Port"], port)

    def test_ioc_unique_resources_and_pwm_parameters_match_source(self):
        keys = [key for key, _ in self.ioc_lines]
        self.assertEqual(len(keys), len(set(keys)), "Duplicate IOC keys")
        for resource, count_key in (("Pin", "Mcu.PinsNb"), ("IP", "Mcu.IPNb")):
            items = {
                int(match.group(1)): value
                for key, value in self.ioc_lines
                if (match := re.fullmatch(r"Mcu\." + resource + r"(\d+)", key))
            }
            self.assertEqual(set(items), set(range(int(self.ioc[count_key]))))
            self.assertEqual(len(items), len(set(items.values())), resource)
        pins = {value for key, value in self.ioc_lines if re.fullmatch(r"Mcu.Pin\d+", key)}
        self.assertTrue({"PB0", "PB1", "PB9", "PC12", "PC8"}.issubset(pins))
        self.assertNotIn("PC9", pins)
        self.assertNotIn("PA15", pins)
        for pin in ("PB0", "PB1", "PC12"):
            self.assertEqual(self.ioc[pin + ".Signal"], "GPIO_Output")
            self.assertEqual(self.ioc[pin + ".PinState"], "GPIO_PIN_RESET")
            self.assertEqual(self.ioc[pin + ".GPIO_PuPd"], "GPIO_PULLDOWN")
            self.assertEqual(self.ioc[pin + ".GPIO_ModeDefaultOutputPP"], "GPIO_MODE_OUTPUT_PP")
        self.assertEqual(self.ioc["PB9.Signal"], "S_TIM11_CH1")
        self.assertEqual(self.ioc["PB9.GPIO_PuPd"], "GPIO_PULLDOWN")
        self.assertEqual(self.ioc["SH.S_TIM11_CH1.0"], "TIM11_CH1,PWM Generation1 CH1")
        body = function_body(self.timer, "MX_TIM11_Init")
        for parameter, expected in (("Prescaler", 83), ("Period", 199)):
            match = re.search(r"htim11\.Init\." + parameter + r"\s*=\s*(\d+)\s*;", body)
            self.assertIsNotNone(match)
            self.assertEqual(int(match.group(1)), expected)
            self.assertEqual(int(self.ioc["TIM11." + parameter]), expected)
        frequency = int(self.ioc["RCC.APB2TimFreq_Value"])
        frequency /= (int(self.ioc["TIM11.Prescaler"]) + 1) * (int(self.ioc["TIM11.Period"]) + 1)
        self.assertEqual(frequency, 10000)

    def test_gpio_pwm_safe_init_and_no_timer_irq(self):
        gpio = function_body(self.gpio, "MX_GPIO_Init")
        first_init = gpio.index("HAL_GPIO_Init(")
        for reset_pattern in (
            r"HAL_GPIO_WritePin\(GPIOB,\s*LASER_AIN1_Pin\s*\|\s*LASER_AIN2_Pin\s*\|\s*LASER_PWMA_Pin,\s*GPIO_PIN_RESET\)",
            r"HAL_GPIO_WritePin\(GPIOC,\s*GPIO_PIN_8\s*\|\s*LASER_STBY_Pin,\s*GPIO_PIN_RESET\)",
        ):
            match = re.search(reset_pattern, gpio)
            self.assertIsNotNone(match)
            self.assertLess(match.start(), first_init, "Preload OFF before enabling outputs")
        self.assertRegex(gpio, r"GPIO_InitStruct\.Pin\s*=\s*LASER_AIN1_Pin\|LASER_AIN2_Pin;\s*GPIO_InitStruct\.Mode\s*=\s*GPIO_MODE_OUTPUT_PP;\s*GPIO_InitStruct\.Pull\s*=\s*GPIO_PULLDOWN;")
        self.assertRegex(gpio, r"GPIO_InitStruct\.Pin\s*=\s*LASER_STBY_Pin;\s*HAL_GPIO_Init\(LASER_STBY_GPIO_Port,\s*&GPIO_InitStruct\)")
        postinit = function_body(self.timer, "HAL_TIM_MspPostInit")
        match = re.search(r"else\s+if\s*\(timHandle->Instance\s*==\s*TIM11\)\s*\{", postinit)
        self.assertIsNotNone(match)
        laser_post = brace_body(postinit, match.end() - 1)
        for statement in (
            "GPIO_InitStruct.Pin = LASER_PWMA_Pin;",
            "GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;",
            "GPIO_InitStruct.Pull = GPIO_PULLDOWN;",
            "GPIO_InitStruct.Alternate = GPIO_AF3_TIM11;",
            "HAL_GPIO_Init(LASER_PWMA_GPIO_Port, &GPIO_InitStruct);",
        ):
            self.assertIn(statement, laser_post)
        timer_init = function_body(self.timer, "MX_TIM11_Init")
        self.assertIn("sConfigOC.Pulse = 0;", timer_init)
        self.assertIn("sConfigOC.OCMode = TIM_OCMODE_PWM1;", timer_init)
        self.assertIn("sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;", timer_init)
        self.assertNotIn("HAL_TIM_PWM_Start", timer_init)
        main = function_body(self.main, "main")
        positions = [main.index(call) for call in (
            "MX_GPIO_Init();", "MX_TIM11_Init();", "robot_init();", "osKernelInitialize();"
        )]
        self.assertEqual(positions, sorted(positions))
        self.assertNotRegex(without_comments(self.irq), r"\bTIM1_TRG_COM_TIM11_IRQHandler\s*\(")
        self.assertFalse(any("TIM11" in key for key in self.ioc if key.startswith("NVIC.")))
        timer_msp = function_body(self.timer, "HAL_TIM_PWM_MspInit")
        match = re.search(r"else\s+if\s*\(tim_pwmHandle->Instance\s*==\s*TIM11\)\s*\{", timer_msp)
        self.assertIsNotNone(match)
        self.assertNotIn("HAL_NVIC", brace_body(timer_msp, match.end() - 1))

    def test_no_old_pa15_laser_output_in_current_source(self):
        for folder in ("App", "Src", "Inc"):
            for file in (ROOT / folder).iterdir():
                if file.suffix not in (".c", ".h"):
                    continue
                text = without_comments(file.read_text(encoding="utf-8-sig"))
                with self.subTest(file=str(file.relative_to(ROOT))):
                    self.assertNotIn("PA15", text)
                    self.assertNotIn("PIN_LASER", text)
                    self.assertNotRegex(text, r"HAL_GPIO_(?:WritePin|TogglePin)\(\s*GPIOA,\s*[^;]*GPIO_PIN_15")
                    self.assertNotRegex(text, r"GPIOA->BSRR\s*=\s*[^;]*GPIO_PIN_15")
        self.assertIn("bp_laser_set(int on)", self.board)

    def test_all_fatal_paths_use_handle_independent_laser_off(self):
        for text, helper in ((self.main, "Error_Handler"), (self.irq, "fault_outputs_off"), (self.rtos, "rtos_emergency_stop")):
            with self.subTest(helper=helper):
                body = function_body(text, helper)
                self.assertIn("bp_laser_emergency_off();", body)
                self.assertRegex(body, r"GPIOC->BSRR\s*=\s*\(uint32_t\)GPIO_PIN_8\s*<<\s*16u;")
                self.assertNotIn("bp_laser_set(", body)
                self.assertNotIn("HAL_GPIO_WritePin", body)
        for handler in ("NMI_Handler", "HardFault_Handler", "MemManage_Handler", "BusFault_Handler", "UsageFault_Handler"):
            body = function_body(self.irq, handler)
            self.assertEqual(body.count("fault_outputs_off();"), 1)
            self.assertLess(body.index("fault_outputs_off();"), body.index("while"))
        for hook in ("vApplicationStackOverflowHook", "vApplicationMallocFailedHook"):
            self.assertEqual(function_body(self.rtos, hook).count("rtos_emergency_stop();"), 1)
        emergency = function_body(self.board, "bp_laser_emergency_off")
        self.assertIn("s_laser_ready = 0u;", emergency)
        self.assertNotIn("htim11", emergency)
        self.assertNotRegex(emergency, r"\bHAL_(?:GPIO|TIM|UART|Delay|GetTick)")
        self.assertNotRegex(emergency, r"\b(?:osDelay|taskENTER_CRITICAL|bp_debug_send)\s*\(")
        self.assertLess(emergency.index("LASER_STBY_GPIO_Port->BSRR"), emergency.index("TIM11->CCR1 = 0u;"))
        self.assertIn("TIM11->CCER &= ~TIM_CCER_CC1E;", emergency)
        self.assertIn("TIM11->CR1 &= ~TIM_CR1_CEN;", emergency)


if __name__ == "__main__":
    unittest.main(verbosity=2)
