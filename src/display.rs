use anyhow::anyhow;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, StrokeAlignment};
use embedded_graphics::text::Text;
use esp32_hal::i2c::I2C;
use esp32_hal::peripherals::I2C0;
use log::info;
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::{DisplayConfig, I2CInterface};
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use static_cell::StaticCell;

pub(crate) struct DisplayController {
    display: &'static mut Ssd1306<
        I2CInterface<I2C<'static, I2C0>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
}

static DISPLAY: StaticCell<
    Ssd1306<
        I2CInterface<I2C<'_, I2C0>>,
        DisplaySize128x64,
        BufferedGraphicsMode<DisplaySize128x64>,
    >,
> = StaticCell::new();

impl DisplayController {
    pub(crate) fn from_i2c(i2c: I2C<'static, I2C0>) -> anyhow::Result<Self> {
        let interface = I2CDisplayInterface::new(i2c);
        let display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        let display = DISPLAY.init(display);

        display
            .init()
            .map_err(|err| anyhow::anyhow!("could not initialize display: {err:?}"))?;
        display.clear();
        Ok(Self { display })
    }

    /// Draw a message to the display.
    pub(crate) fn draw_message(&mut self, text: &str) -> anyhow::Result<()> {
        info!("drawing message to display: {text}");
        self.display.clear();
        self.draw_border()?;
        self.draw_title(text)?;
        self.display
            .flush()
            .map_err(|err| anyhow!("failed to flush display buffer to display: {err:?}"))?;
        Ok(())
    }

    fn draw_border(&mut self) -> anyhow::Result<()> {
        let style = PrimitiveStyleBuilder::new()
            .stroke_color(BinaryColor::On)
            .stroke_width(3)
            .stroke_alignment(StrokeAlignment::Inside)
            .build();
        self.display
            .bounding_box()
            .into_styled(style)
            .draw(self.display)
            .map_err(|err| anyhow!("failed to draw border: {err:?}"))?;
        Ok(())
    }

    fn draw_title(&mut self, text: &str) -> anyhow::Result<()> {
        let position = self.display.bounding_box().top_left + Point::new(10, 20);
        let character_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        let text = Text::with_alignment(
            text,
            position,
            character_style,
            embedded_graphics::text::Alignment::Left,
        );
        text.draw(self.display)
            .map_err(|err| anyhow!("failed to draw footer message: {err:?}"))?;

        Ok(())
    }
}
