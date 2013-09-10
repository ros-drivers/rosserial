extern void init(void);
extern void setup(void);
extern void loop(void);

int main( void )
{
	init();

	setup();

	for (;;)
	{
		loop();
	}

	return 0;
}
